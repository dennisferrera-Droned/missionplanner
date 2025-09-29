package com.dji.photogrammetry

import com.google.gson.Gson
import com.google.gson.annotations.SerializedName
import java.util.*
import kotlin.math.*

/**
 * Camera trigger system for DJI drones with GPS/distance-based automation.
 * 
 * Handles automatic camera triggering based on time intervals, distance traveled,
 * or GPS coordinates. Logs all triggers with exact GPS position, altitude,
 * drone attitude, timestamp, and camera settings.
 */

enum class TriggerMode {
    @SerializedName("time")
    TIME_BASED,
    @SerializedName("distance") 
    DISTANCE_BASED,
    @SerializedName("waypoint")
    WAYPOINT_BASED,
    @SerializedName("manual")
    MANUAL
}

data class DroneState(
    val latitude: Double,
    val longitude: Double,
    val altitudeM: Double,
    val headingDeg: Double,
    val pitchDeg: Double,
    val rollDeg: Double,
    val yawDeg: Double,
    val groundSpeedMs: Double,
    val timestamp: Date
)

data class CameraState(
    val iso: Int = 100,
    val aperture: String = "f/2.8",
    val shutterSpeed: String = "1/500",
    val focalLengthMm: Double = 24.0,
    val whiteBalance: String = "auto",
    val exposureMode: String = "auto",
    val focusMode: String = "infinity",
    val imageFormat: String = "JPEG"
)

data class TriggerEvent(
    val triggerId: String,
    val triggerMode: TriggerMode,
    val droneState: DroneState,
    val cameraState: CameraState,
    val imageFilename: String? = null,
    val triggerReason: String = ""
)

data class MissionSummary(
    val totalTriggers: Int,
    val missionDurationS: Long,
    val totalDistanceM: Double,
    val triggerMode: String,
    val altitudeRange: AltitudeRange,
    val firstTrigger: Date?,
    val lastTrigger: Date?
)

data class AltitudeRange(
    val min: Double,
    val max: Double,
    val avg: Double
)

interface CameraTriggerListener {
    fun onCameraTrigger(): String? // Returns image filename or null if failed
    fun getCurrentDroneState(): DroneState?
    fun getCurrentCameraState(): CameraState?
}

class CameraTrigger {
    private val triggerEvents = mutableListOf<TriggerEvent>()
    private var lastTriggerTime: Date? = null
    private var lastTriggerPosition: Pair<Double, Double>? = null
    private var triggerCount = 0
    private var isActive = false
    
    // Trigger settings
    private var timeIntervalS: Double? = null
    private var distanceIntervalM: Double? = null
    private var currentMode = TriggerMode.MANUAL
    
    // Listener for DJI SDK integration
    private var listener: CameraTriggerListener? = null
    
    fun setTriggerMode(mode: TriggerMode, vararg params: Pair<String, Any>) {
        currentMode = mode
        
        when (mode) {
            TriggerMode.TIME_BASED -> {
                timeIntervalS = params.find { it.first == "timeIntervalS" }?.second as? Double ?: 2.0
                distanceIntervalM = null
            }
            TriggerMode.DISTANCE_BASED -> {
                distanceIntervalM = params.find { it.first == "distanceIntervalM" }?.second as? Double ?: 30.0
                timeIntervalS = null
            }
            TriggerMode.WAYPOINT_BASED, TriggerMode.MANUAL -> {
                timeIntervalS = null
                distanceIntervalM = null
            }
        }
    }
    
    fun setListener(listener: CameraTriggerListener) {
        this.listener = listener
    }
    
    fun startTriggering() {
        isActive = true
        lastTriggerTime = null
        lastTriggerPosition = null
        println("Camera triggering started in ${currentMode.name} mode")
    }
    
    fun stopTriggering() {
        isActive = false
        println("Camera triggering stopped. Total triggers: $triggerCount")
    }
    
    /**
     * Update trigger system with current drone state.
     * Should be called regularly during flight to check trigger conditions.
     */
    fun update(): Boolean {
        if (!isActive || listener == null) return false
        
        val currentDroneState = listener?.getCurrentDroneState() ?: return false
        val currentTime = Date()
        
        val (shouldTrigger, triggerReason) = when (currentMode) {
            TriggerMode.TIME_BASED -> checkTimeTrigger(currentTime)
            TriggerMode.DISTANCE_BASED -> checkDistanceTrigger(currentDroneState)
            else -> Pair(false, "")
        }
        
        return if (shouldTrigger) {
            triggerCamera(currentDroneState, triggerReason)
        } else false
    }
    
    private fun checkTimeTrigger(currentTime: Date): Pair<Boolean, String> {
        val interval = timeIntervalS ?: return Pair(false, "")
        
        val lastTime = lastTriggerTime
        if (lastTime == null) {
            return Pair(true, "Initial time-based trigger (interval: ${interval}s)")
        }
        
        val timeElapsed = (currentTime.time - lastTime.time) / 1000.0
        return if (timeElapsed >= interval) {
            Pair(true, "Time interval reached (${timeElapsed}s >= ${interval}s)")
        } else {
            Pair(false, "")
        }
    }
    
    private fun checkDistanceTrigger(droneState: DroneState): Pair<Boolean, String> {
        val interval = distanceIntervalM ?: return Pair(false, "")
        
        val currentPos = Pair(droneState.latitude, droneState.longitude)
        val lastPos = lastTriggerPosition
        
        if (lastPos == null) {
            return Pair(true, "Initial distance-based trigger (interval: ${interval}m)")
        }
        
        val distanceTraveled = calculateDistance(lastPos, currentPos)
        return if (distanceTraveled >= interval) {
            Pair(true, "Distance interval reached (${distanceTraveled}m >= ${interval}m)")
        } else {
            Pair(false, "")
        }
    }
    
    private fun calculateDistance(pos1: Pair<Double, Double>, pos2: Pair<Double, Double>): Double {
        val (lat1, lon1) = pos1
        val (lat2, lon2) = pos2
        
        // Convert to radians
        val lat1Rad = Math.toRadians(lat1)
        val lon1Rad = Math.toRadians(lon1)
        val lat2Rad = Math.toRadians(lat2)
        val lon2Rad = Math.toRadians(lon2)
        
        // Haversine formula
        val dlat = lat2Rad - lat1Rad
        val dlon = lon2Rad - lon1Rad
        
        val a = sin(dlat / 2).pow(2) + cos(lat1Rad) * cos(lat2Rad) * sin(dlon / 2).pow(2)
        val c = 2 * asin(sqrt(a))
        
        // Earth radius in meters
        val earthRadiusM = 6371000.0
        return earthRadiusM * c
    }
    
    /**
     * Trigger camera capture and log metadata.
     */
    fun triggerCamera(droneState: DroneState? = null, reason: String = "Manual trigger"): Boolean {
        val currentListener = listener ?: return false
        
        // Get current states
        val currentDroneState = droneState ?: currentListener.getCurrentDroneState() ?: return false
        val cameraState = currentListener.getCurrentCameraState() ?: CameraState()
        
        // Trigger camera
        val imageFilename = try {
            currentListener.onCameraTrigger()
        } catch (e: Exception) {
            println("Camera trigger failed: ${e.message}")
            return false
        }
        
        // Create trigger event
        triggerCount++
        val triggerId = "trigger_${triggerCount.toString().padStart(6, '0')}_${System.currentTimeMillis()}"
        
        val triggerEvent = TriggerEvent(
            triggerId = triggerId,
            triggerMode = currentMode,
            droneState = currentDroneState,
            cameraState = cameraState,
            imageFilename = imageFilename,
            triggerReason = reason
        )
        
        triggerEvents.add(triggerEvent)
        
        // Update trigger history
        lastTriggerTime = currentDroneState.timestamp
        lastTriggerPosition = Pair(currentDroneState.latitude, currentDroneState.longitude)
        
        println("Camera triggered: $triggerId - $reason")
        return true
    }
    
    /**
     * Trigger camera at specific waypoint.
     */
    fun triggerAtWaypoint(waypointId: String, droneState: DroneState? = null): Boolean {
        val reason = "Waypoint trigger: $waypointId"
        return triggerCamera(droneState, reason)
    }
    
    /**
     * Export trigger metadata to JSON string.
     */
    fun exportMetadataJson(): String {
        val exportData = mapOf(
            "mission_info" to mapOf(
                "total_triggers" to triggerEvents.size,
                "trigger_mode" to currentMode.name,
                "export_timestamp" to Date().toString()
            ),
            "trigger_events" to triggerEvents
        )
        
        return Gson().toJson(exportData)
    }
    
    /**
     * Get mission summary statistics.
     */
    fun getMissionSummary(): MissionSummary {
        if (triggerEvents.isEmpty()) {
            return MissionSummary(
                totalTriggers = 0,
                missionDurationS = 0,
                totalDistanceM = 0.0,
                triggerMode = currentMode.name,
                altitudeRange = AltitudeRange(0.0, 0.0, 0.0),
                firstTrigger = null,
                lastTrigger = null
            )
        }
        
        // Calculate mission duration
        val firstTrigger = triggerEvents.first().droneState.timestamp
        val lastTrigger = triggerEvents.last().droneState.timestamp
        val durationS = (lastTrigger.time - firstTrigger.time) / 1000
        
        // Calculate total distance traveled
        var totalDistance = 0.0
        for (i in 1 until triggerEvents.size) {
            val pos1 = Pair(
                triggerEvents[i - 1].droneState.latitude,
                triggerEvents[i - 1].droneState.longitude
            )
            val pos2 = Pair(
                triggerEvents[i].droneState.latitude,
                triggerEvents[i].droneState.longitude
            )
            totalDistance += calculateDistance(pos1, pos2)
        }
        
        // Get altitude range
        val altitudes = triggerEvents.map { it.droneState.altitudeM }
        val altitudeRange = AltitudeRange(
            min = altitudes.minOrNull() ?: 0.0,
            max = altitudes.maxOrNull() ?: 0.0,
            avg = altitudes.average()
        )
        
        return MissionSummary(
            totalTriggers = triggerEvents.size,
            missionDurationS = durationS,
            totalDistanceM = totalDistance,
            triggerMode = currentMode.name,
            altitudeRange = altitudeRange,
            firstTrigger = firstTrigger,
            lastTrigger = lastTrigger
        )
    }
    
    /**
     * Clear all trigger events and reset counters.
     */
    fun reset() {
        triggerEvents.clear()
        triggerCount = 0
        lastTriggerTime = null
        lastTriggerPosition = null
        isActive = false
    }
    
    /**
     * Get all trigger events.
     */
    fun getTriggerEvents(): List<TriggerEvent> = triggerEvents.toList()
}