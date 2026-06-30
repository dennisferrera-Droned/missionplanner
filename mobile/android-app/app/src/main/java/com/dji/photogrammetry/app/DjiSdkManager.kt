package com.dji.photogrammetry.app

import android.content.Context
import android.util.Log
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.common.key.BatteryKey
import dji.v5.common.key.FlightControllerKey
import dji.v5.common.key.KeyTools
import dji.v5.common.key.ProductKey
import dji.v5.manager.KeyManager
import dji.v5.manager.SDKManager
import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
import dji.v5.manager.aircraft.waypoint3.model.ActionWhenFinished
import dji.v5.manager.aircraft.waypoint3.model.MissionConfig
import dji.v5.manager.aircraft.waypoint3.model.WaypointInfo
import dji.v5.manager.aircraft.waypoint3.model.WaypointMission
import dji.v5.manager.interfaces.SDKManagerCallback
import dji.v5.utils.common.LocationCoordinate2D

/**
 * Singleton that owns the DJI MSDK v5 lifecycle.
 *
 * Exposes aircraft telemetry as [LiveData] so any Activity/Fragment can
 * observe without worrying about SDK listener registration/cleanup.
 *
 * Usage
 * -----
 * Initialise once in [App.onCreate]:
 *   DjiSdkManager.init(context)
 *
 * Observe from any Activity:
 *   DjiSdkManager.isConnected.observe(this) { connected -> … }
 *
 * Upload / control a mission:
 *   DjiSdkManager.uploadMission(mission, onResult)
 *   DjiSdkManager.startMission(onResult)
 *   DjiSdkManager.pauseMission(onResult)
 *   DjiSdkManager.stopMission(onResult)
 */
object DjiSdkManager {

    private const val TAG = "DjiSdkManager"

    // ── Observable state ────────────────────────────────────────────────────

    /** true once the SDK has registered with DJI servers. */
    private val _isRegistered = MutableLiveData(false)
    val isRegistered: LiveData<Boolean> = _isRegistered

    /** true when an aircraft is physically connected. */
    private val _isConnected = MutableLiveData(false)
    val isConnected: LiveData<Boolean> = _isConnected

    /** 0–100 %. -1 when unknown. */
    private val _batteryPercent = MutableLiveData(-1)
    val batteryPercent: LiveData<Int> = _batteryPercent

    /** GPS signal quality 0 (none) – 5 (excellent). */
    private val _gpsSignal = MutableLiveData(0)
    val gpsSignal: LiveData<Int> = _gpsSignal

    /** Number of GPS satellites locked. */
    private val _gpsSatellites = MutableLiveData(0)
    val gpsSatellites: LiveData<Int> = _gpsSatellites

    /** Null when no location fix. Pair(latitude, longitude). */
    private val _droneLocation = MutableLiveData<Pair<Double, Double>?>(null)
    val droneLocation: LiveData<Pair<Double, Double>?> = _droneLocation

    /** Altitude above take-off point, metres. */
    private val _altitude = MutableLiveData(0.0)
    val altitude: LiveData<Double> = _altitude

    /** Human-readable flight/mission status string. */
    private val _statusText = MutableLiveData("SDK not initialised")
    val statusText: LiveData<String> = _statusText

    // ── Initialisation ──────────────────────────────────────────────────────

    fun init(context: Context) {
        _statusText.postValue("Registering SDK…")
        SDKManager.getInstance().init(context, object : SDKManagerCallback {

            override fun onRegisterSuccess() {
                Log.i(TAG, "SDK registered ✓")
                _isRegistered.postValue(true)
                _statusText.postValue("SDK registered — waiting for aircraft")
                startKeyListeners()
            }

            override fun onRegisterFailure(error: IDJIError) {
                Log.e(TAG, "SDK register failed: ${error.description()}")
                _isRegistered.postValue(false)
                _statusText.postValue("SDK registration failed: ${error.description()}")
            }

            override fun onProductConnect(productType: dji.v5.manager.interfaces.ProductType) {
                Log.i(TAG, "Aircraft connected: $productType")
                _isConnected.postValue(true)
                _statusText.postValue("Connected: $productType")
            }

            override fun onProductDisconnect(productType: dji.v5.manager.interfaces.ProductType) {
                Log.i(TAG, "Aircraft disconnected")
                _isConnected.postValue(false)
                _droneLocation.postValue(null)
                _batteryPercent.postValue(-1)
                _gpsSignal.postValue(0)
                _statusText.postValue("Aircraft disconnected")
            }

            override fun onProductChanged(productType: dji.v5.manager.interfaces.ProductType) {
                Log.i(TAG, "Product changed: $productType")
            }

            override fun onInitProcess(
                event: dji.v5.manager.interfaces.DJISDKInitEvent,
                totalProcess: Int
            ) {
                Log.d(TAG, "Init process: $event ($totalProcess %)")
                _statusText.postValue("Initialising… $totalProcess%")
            }

            override fun onDatabaseDownloadProgress(current: Long, total: Long) {
                val pct = if (total > 0) (current * 100 / total).toInt() else 0
                Log.d(TAG, "Geofence DB download: $pct%")
                _statusText.postValue("Downloading geofence DB: $pct%")
            }
        })
    }

    // ── Key listeners ───────────────────────────────────────────────────────

    private fun startKeyListeners() {
        val km = KeyManager.getInstance()

        // Aircraft GPS position
        km.listen(
            KeyTools.createKey(FlightControllerKey.KeyAircraftLocation3D)
        ) { _, loc ->
            loc?.let {
                _droneLocation.postValue(Pair(it.latitude, it.longitude))
                _altitude.postValue(it.altitude.toDouble())
            }
        }

        // Battery
        km.listen(
            KeyTools.createKey(BatteryKey.KeyChargeRemainingInPercent)
        ) { _, percent ->
            percent?.let { _batteryPercent.postValue(it) }
        }

        // GPS signal quality (enum ordinal = 0..5)
        km.listen(
            KeyTools.createKey(FlightControllerKey.KeyGPSSignalLevel)
        ) { _, level ->
            level?.let { _gpsSignal.postValue(it.ordinal) }
        }

        // GPS satellite count
        km.listen(
            KeyTools.createKey(FlightControllerKey.KeyGPSSatelliteCount)
        ) { _, count ->
            count?.let { _gpsSatellites.postValue(it) }
        }

        // Product connection (redundant safety net besides the callback above)
        km.listen(
            KeyTools.createKey(ProductKey.KeyConnection)
        ) { _, connected ->
            connected?.let { _isConnected.postValue(it) }
        }
    }

    // ── Mission control ─────────────────────────────────────────────────────

    /**
     * Upload a [WaypointMission] to the aircraft.
     * [onResult] receives null on success or an error message on failure.
     */
    fun uploadMission(mission: WaypointMission, onResult: (error: String?) -> Unit) {
        _statusText.postValue("Uploading mission…")
        WaypointMissionManager.getInstance().uploadMission(
            mission,
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    _statusText.postValue("Mission uploaded ✓ — ready to start")
                    onResult(null)
                }
                override fun onFailure(error: IDJIError) {
                    val msg = error.description()
                    _statusText.postValue("Upload failed: $msg")
                    onResult(msg)
                }
            }
        )
    }

    fun startMission(onResult: (error: String?) -> Unit) {
        _statusText.postValue("Starting mission…")
        WaypointMissionManager.getInstance().startMission(
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    _statusText.postValue("Mission running ▶")
                    onResult(null)
                }
                override fun onFailure(error: IDJIError) {
                    val msg = error.description()
                    _statusText.postValue("Start failed: $msg")
                    onResult(msg)
                }
            }
        )
    }

    fun pauseMission(onResult: (error: String?) -> Unit) {
        WaypointMissionManager.getInstance().pauseMission(
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    _statusText.postValue("Mission paused ⏸")
                    onResult(null)
                }
                override fun onFailure(error: IDJIError) {
                    onResult(error.description())
                }
            }
        )
    }

    fun stopMission(onResult: (error: String?) -> Unit) {
        WaypointMissionManager.getInstance().stopMission(
            object : CommonCallbacks.CompletionCallback {
                override fun onSuccess() {
                    _statusText.postValue("Mission stopped ⏹")
                    onResult(null)
                }
                override fun onFailure(error: IDJIError) {
                    onResult(error.description())
                }
            }
        )
    }

    // ── Helpers ─────────────────────────────────────────────────────────────

    /**
     * Build a [WaypointMission] from a list of (lat, lon, altM) triples and
     * a flight speed.  Used by [MissionConverter] and [FlightControlActivity].
     */
    fun buildMission(
        points: List<Triple<Double, Double, Double>>,
        speedMs: Float = 5f
    ): WaypointMission {
        val waypointInfoList = points.map { (lat, lon, alt) ->
            WaypointInfo.Builder()
                .coordinate(LocationCoordinate2D(lat, lon))
                .altitude(alt.toFloat())
                .build()
        }

        val config = MissionConfig.Builder()
            .finishAction(ActionWhenFinished.NO_ACTION)
            .autoFlightSpeed(speedMs)
            .maxFlightSpeed(15f)
            .exitMissionOnRCSignalLost(true)
            .build()

        return WaypointMission.Builder()
            .missionConfig(config)
            .waypointInfoList(waypointInfoList)
            .build()
    }
}
