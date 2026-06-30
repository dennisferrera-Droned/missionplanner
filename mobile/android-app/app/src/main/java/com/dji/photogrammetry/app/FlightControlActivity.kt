package com.dji.photogrammetry.app

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityFlightControlBinding
import dji.v5.manager.aircraft.waypoint3.model.WaypointMission
import org.osmdroid.config.Configuration
import org.osmdroid.tileprovider.tilesource.TileSourceFactory
import org.osmdroid.util.GeoPoint
import org.osmdroid.views.overlay.Marker
import org.osmdroid.views.overlay.Polyline

/**
 * Flight Control cockpit:
 *  • Live map (OsmDroid / OpenStreetMap — no API key required)
 *  • Aircraft status: connection, GPS signal, satellite count, battery
 *  • Mission load from JSON file or passed via Intent extra
 *  • Upload → Start → Pause / Stop mission controls
 *
 * The screen is designed for landscape orientation to give the map as much
 * real-estate as possible while keeping controls on the right-hand panel.
 *
 * ── DJI SDK note ────────────────────────────────────────────────────────────
 * All hardware interaction goes through [DjiSdkManager], which is initialised
 * once in [App.onCreate].  If the SDK is not yet registered (e.g. the device
 * is offline and the geofence database hasn't been downloaded), the status
 * panel will show the current progress automatically.
 */
class FlightControlActivity : AppCompatActivity() {

    private lateinit var binding: ActivityFlightControlBinding

    // The currently loaded, parsed DJI mission (null = nothing loaded yet)
    private var currentMission: WaypointMission? = null

    // OsmDroid markers
    private var droneMarker: Marker? = null
    private var missionPolyline: Polyline? = null

    // Track whether we are paused (to toggle button labels)
    private var isPaused = false

    // ── File picker ──────────────────────────────────────────────────────────
    private val jsonPickerLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        val uri = result.data?.data ?: return@registerForActivityResult
        loadMissionFromUri(uri)
    }

    // ── Lifecycle ────────────────────────────────────────────────────────────

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // OsmDroid requires user-agent before the MapView is inflated
        Configuration.getInstance().load(this, getPreferences(MODE_PRIVATE))
        Configuration.getInstance().userAgentValue = packageName

        binding = ActivityFlightControlBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        setupMap()
        observeSdk()
        setupButtons()

        // Accept a pre-built mission JSON passed from MissionPlannerActivity / RouteFollowActivity
        intent.getStringExtra(EXTRA_MISSION_JSON)?.let { json ->
            parseMissionJson(json)
        }
    }

    override fun onResume() {
        super.onResume()
        binding.mapView.onResume()
    }

    override fun onPause() {
        super.onPause()
        binding.mapView.onPause()
    }

    // ── Map setup ────────────────────────────────────────────────────────────

    private fun setupMap() {
        binding.mapView.apply {
            setTileSource(TileSourceFactory.MAPNIK)   // OpenStreetMap tiles
            setMultiTouchControls(true)
            controller.setZoom(16.0)
            controller.setCenter(GeoPoint(47.3769, 8.5417))  // default: Zurich
        }
    }

    /** Move the drone marker to a new position and re-centre the map. */
    private fun updateDroneOnMap(lat: Double, lon: Double) {
        val point = GeoPoint(lat, lon)

        if (droneMarker == null) {
            droneMarker = Marker(binding.mapView).apply {
                title = "Drone"
                setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_BOTTOM)
            }
            binding.mapView.overlays.add(droneMarker)
        }
        droneMarker!!.position = point
        binding.mapView.controller.animateTo(point)
        binding.mapView.invalidate()
    }

    /** Draw the mission waypoints as a polyline on the map. */
    private fun drawMissionOnMap(waypoints: List<GeoPoint>) {
        // Remove old line
        missionPolyline?.let { binding.mapView.overlays.remove(it) }

        if (waypoints.isEmpty()) return

        missionPolyline = Polyline().apply {
            setPoints(waypoints)
            outlinePaint.color = android.graphics.Color.parseColor("#2196F3")
            outlinePaint.strokeWidth = 4f
        }
        binding.mapView.overlays.add(missionPolyline)

        // Centre the map on the first waypoint
        binding.mapView.controller.animateTo(waypoints.first())
        binding.mapView.invalidate()
    }

    // ── SDK observation ──────────────────────────────────────────────────────

    private fun observeSdk() {
        // Connection status
        DjiSdkManager.isConnected.observe(this) { connected ->
            if (connected) {
                binding.tvConnection.text      = "Connected ✓"
                binding.tvConnection.setTextColor(getColor(android.R.color.holo_green_dark))
            } else {
                binding.tvConnection.text      = "Disconnected"
                binding.tvConnection.setTextColor(getColor(android.R.color.holo_red_dark))
                binding.tvGps.text             = "—"
                binding.tvSatellites.text      = "—"
                binding.tvBattery.text         = "—"
                binding.tvMapCoords.text       = "—"
                binding.tvMapAlt.text          = "Alt: —"
            }
            refreshControlButtons(connected)
        }

        // GPS signal level (0–5)
        DjiSdkManager.gpsSignal.observe(this) { level ->
            val bars = "▮".repeat(level) + "▯".repeat(5 - level)
            binding.tvGps.text = "$bars  (L$level)"
        }

        // Satellite count
        DjiSdkManager.gpsSatellites.observe(this) { count ->
            binding.tvSatellites.text = "$count"
        }

        // Battery
        DjiSdkManager.batteryPercent.observe(this) { pct ->
            binding.tvBattery.text = if (pct < 0) "—" else {
                val emoji = when {
                    pct >= 60 -> "🟢"
                    pct >= 30 -> "🟡"
                    else      -> "🔴"
                }
                "$emoji $pct %"
            }
        }

        // Drone GPS position
        DjiSdkManager.droneLocation.observe(this) { location ->
            if (location != null) {
                val (lat, lon) = location
                updateDroneOnMap(lat, lon)
                binding.tvMapCoords.text = "%.6f, %.6f".format(lat, lon)
            }
        }

        // Altitude
        DjiSdkManager.altitude.observe(this) { alt ->
            binding.tvMapAlt.text = "Alt: ${"%.1f".format(alt)} m"
        }

        // SDK / mission status messages
        DjiSdkManager.statusText.observe(this) { msg ->
            binding.tvSdkStatus.text = msg
        }
    }

    // ── Button setup ─────────────────────────────────────────────────────────

    private fun setupButtons() {
        binding.btnLoadMission.setOnClickListener {
            val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                addCategory(Intent.CATEGORY_OPENABLE)
                type = "application/json"
                putExtra(
                    Intent.EXTRA_MIME_TYPES,
                    arrayOf("application/json", "text/plain", "application/octet-stream")
                )
            }
            jsonPickerLauncher.launch(intent)
        }

        binding.btnUpload.setOnClickListener {
            val mission = currentMission
            if (mission == null) {
                Toast.makeText(this, "Load a mission JSON first", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            binding.btnUpload.isEnabled = false
            DjiSdkManager.uploadMission(mission) { error ->
                runOnUiThread {
                    binding.btnUpload.isEnabled = true
                    if (error == null) {
                        binding.btnStart.isEnabled = true
                        binding.btnStop.isEnabled  = true
                    } else {
                        showError("Upload failed", error)
                    }
                }
            }
        }

        binding.btnStart.setOnClickListener {
            if (isPaused) {
                // Resume (DJI SDK: start after pause resumes the mission)
                DjiSdkManager.startMission { error ->
                    runOnUiThread {
                        if (error == null) {
                            isPaused = false
                            binding.btnStart.text  = "▶ Start"
                            binding.btnPause.isEnabled = true
                        } else showError("Resume failed", error)
                    }
                }
            } else {
                DjiSdkManager.startMission { error ->
                    runOnUiThread {
                        if (error == null) {
                            binding.btnPause.isEnabled = true
                            binding.btnStart.isEnabled = false
                        } else showError("Start failed", error)
                    }
                }
            }
        }

        binding.btnPause.setOnClickListener {
            DjiSdkManager.pauseMission { error ->
                runOnUiThread {
                    if (error == null) {
                        isPaused = true
                        binding.btnStart.text      = "▶ Resume"
                        binding.btnStart.isEnabled = true
                        binding.btnPause.isEnabled = false
                    } else showError("Pause failed", error)
                }
            }
        }

        binding.btnStop.setOnClickListener {
            AlertDialog.Builder(this)
                .setTitle("Stop Mission")
                .setMessage("Are you sure you want to abort the mission?\nThe drone will hover in place.")
                .setPositiveButton("Stop") { _, _ ->
                    DjiSdkManager.stopMission { error ->
                        runOnUiThread {
                            if (error == null) {
                                isPaused = false
                                binding.btnStart.text      = "▶ Start"
                                binding.btnStart.isEnabled = false
                                binding.btnPause.isEnabled = false
                                binding.btnStop.isEnabled  = false
                            } else showError("Stop failed", error)
                        }
                    }
                }
                .setNegativeButton("Cancel", null)
                .show()
        }
    }

    /** Enable / disable mission controls based on connection + loaded mission. */
    private fun refreshControlButtons(connected: Boolean) {
        val hasMission = currentMission != null
        binding.btnUpload.isEnabled = connected && hasMission
        binding.btnStart.isEnabled  = false
        binding.btnPause.isEnabled  = false
        binding.btnStop.isEnabled   = false
    }

    // ── Mission loading ──────────────────────────────────────────────────────

    private fun loadMissionFromUri(uri: Uri) {
        try {
            val json = contentResolver.openInputStream(uri)
                ?.bufferedReader()?.readText()
                ?: throw IllegalStateException("Cannot read file")
            parseMissionJson(json)
        } catch (e: Exception) {
            showError("Load failed", e.message ?: "Unknown error")
        }
    }

    private fun parseMissionJson(json: String) {
        try {
            val mission = MissionConverter.fromJson(json)
            currentMission = mission
            val count = MissionConverter.waypointCount(json)
            binding.tvMissionSummary.text = "✅ $count waypoints loaded"

            // Draw waypoints on map by re-parsing coordinates from JSON
            drawWaypointsFromJson(json)

            // Enable upload if already connected
            val connected = DjiSdkManager.isConnected.value == true
            binding.btnUpload.isEnabled = connected
        } catch (e: Exception) {
            showError("Invalid mission JSON", e.message ?: "Parse error")
            binding.tvMissionSummary.text = "❌ Invalid JSON"
        }
    }

    private fun drawWaypointsFromJson(json: String) {
        try {
            val root = com.google.gson.JsonParser.parseString(json).asJsonObject
            val array = root.getAsJsonArray("waypoints") ?: return
            val geoPoints = array.mapNotNull { el ->
                val obj = el.asJsonObject
                val lat = obj.get("lat")?.asDouble
                val lon = obj.get("lon")?.asDouble
                if (lat != null && lon != null) GeoPoint(lat, lon) else null
            }
            drawMissionOnMap(geoPoints)
        } catch (_: Exception) { /* ignore display errors */ }
    }

    // ── Helpers ──────────────────────────────────────────────────────────────

    private fun showError(title: String, message: String) {
        AlertDialog.Builder(this)
            .setTitle(title)
            .setMessage(message)
            .setPositiveButton("OK", null)
            .show()
    }

    override fun onSupportNavigateUp(): Boolean { finish(); return true }

    companion object {
        /** Pass a pre-built mission JSON string via this Intent extra. */
        const val EXTRA_MISSION_JSON = "mission_json"
    }
}
