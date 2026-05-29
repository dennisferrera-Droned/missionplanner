package com.dji.photogrammetry

import com.google.gson.Gson
import kotlin.math.*

/**
 * Route-follow / active-track mission controller for DJI drones.
 *
 * Features
 * --------
 * - Pre-defined GPS route that the drone flies at the user's pace.
 * - Speed matching: call setPaceSpeed() whenever the runner/rider changes
 *   speed so the drone stays in frame.
 * - Backwards flight: drone's nose points opposite the direction of travel
 *   so the gimbal can face the user from the front.
 * - Home-point update: when the route ends the drone's RTH destination is
 *   updated to the final waypoint (end of the run/ride).
 *
 * Integration with DJI SDK
 * -------------------------
 * Wire the callbacks:
 *   controller.onVelocityCommand = { vn, ve, vd, yaw ->
 *       flightController.sendVirtualStickFlightControlData(...)
 *   }
 *   controller.onHomeUpdate = { lat, lon ->
 *       flightController.setHomeLocation(LocationCoordinate2D(lat, lon), ...)
 *   }
 *
 * Virtual Stick mode must be enabled on the FlightController before start().
 */

data class RouteWaypoint(
    val latitude: Double,
    val longitude: Double,
    val altitudeM: Double,
    val name: String = ""
)

data class RouteSegment(
    val start: RouteWaypoint,
    val end: RouteWaypoint,
    var speedMs: Double = 5.0,
    var flyBackwards: Boolean = false
) {
    val bearingDeg: Double
        get() {
            val lat1 = Math.toRadians(start.latitude)
            val lat2 = Math.toRadians(end.latitude)
            val dlon = Math.toRadians(end.longitude - start.longitude)
            val x = sin(dlon) * cos(lat2)
            val y = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
            val b = Math.toDegrees(atan2(x, y))
            return (b + 360) % 360
        }

    val lengthM: Double
        get() {
            val lat1 = Math.toRadians(start.latitude)
            val lon1 = Math.toRadians(start.longitude)
            val lat2 = Math.toRadians(end.latitude)
            val lon2 = Math.toRadians(end.longitude)
            val dlat = lat2 - lat1
            val dlon = lon2 - lon1
            val a = sin(dlat / 2).pow(2) + cos(lat1) * cos(lat2) * sin(dlon / 2).pow(2)
            return 6371000.0 * 2 * asin(sqrt(a))
        }

    val durationS: Double get() = if (speedMs > 0) lengthM / speedMs else 0.0

    /**
     * Yaw the drone's nose should point.
     * Forward mode: nose along bearing.
     * Backwards mode: nose opposite bearing (drone reverses while camera faces user).
     */
    val yawDeg: Double
        get() = if (flyBackwards) (bearingDeg + 180) % 360 else bearingDeg

    /**
     * Velocity in North-East-Down frame (m/s).
     * Returns Triple(vNorth, vEast, vDown).
     */
    fun velocityNed(): Triple<Double, Double, Double> {
        val bearingRad = Math.toRadians(bearingDeg)
        val vn = speedMs * cos(bearingRad)
        val ve = speedMs * sin(bearingRad)
        val dalt = end.altitudeM - start.altitudeM
        val vd = if (durationS > 0) -dalt / durationS else 0.0
        return Triple(vn, ve, vd)
    }
}

class FollowRoute(
    val defaultSpeedMs: Double = 5.0,
    val flyBackwards: Boolean = false,
    val updateHome: Boolean = true
) {
    private val _waypoints = mutableListOf<RouteWaypoint>()
    val waypoints: List<RouteWaypoint> get() = _waypoints.toList()

    val finalWaypoint: RouteWaypoint? get() = _waypoints.lastOrNull()

    fun addWaypoint(lat: Double, lon: Double, alt: Double, name: String = "") {
        _waypoints.add(RouteWaypoint(lat, lon, alt, name))
    }

    fun buildSegments(
        perSegmentSpeeds: List<Double>? = null,
        perSegmentBackwards: List<Boolean>? = null
    ): List<RouteSegment> {
        if (_waypoints.size < 2) throw IllegalStateException("Need at least 2 waypoints")
        return (0 until _waypoints.size - 1).map { i ->
            val speed = perSegmentSpeeds?.getOrNull(i) ?: defaultSpeedMs
            val back  = perSegmentBackwards?.getOrNull(i) ?: flyBackwards
            RouteSegment(_waypoints[i], _waypoints[i + 1], speed, back)
        }
    }

    val totalLengthM: Double get() = buildSegments().sumOf { it.lengthM }

    fun toJson(): String = Gson().toJson(
        mapOf(
            "default_speed_ms" to defaultSpeedMs,
            "fly_backwards"    to flyBackwards,
            "update_home"      to updateHome,
            "waypoints"        to _waypoints
        )
    )
}

class FollowModeController(private val route: FollowRoute) {

    private var segments: MutableList<RouteSegment> = mutableListOf()
    private var currentSegIndex = 0
    var isRunning = false
        private set
    var isComplete = false
        private set

    // Wire these callbacks to your DJI SDK calls
    var onVelocityCommand: ((vn: Double, ve: Double, vd: Double, yawDeg: Double) -> Unit)? = null
    var onHomeUpdate: ((lat: Double, lon: Double) -> Unit)? = null

    // Progress
    private val progressLog = mutableListOf<Map<String, Any>>()

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    fun start() {
        segments = route.buildSegments().toMutableList()
        if (segments.isEmpty()) throw IllegalStateException("Route has no segments")
        currentSegIndex = 0
        isRunning = true
        isComplete = false
        println("Follow-mode started: ${segments.size} segments, ${route.totalLengthM.toInt()} m")
    }

    fun stop() {
        isRunning = false
        sendHover()
        println("Follow-mode stopped")
    }

    // -----------------------------------------------------------------------
    // Main tick – call from your GPS update loop (~10 Hz)
    // -----------------------------------------------------------------------

    /**
     * Advance the route.
     *
     * @param currentLat  Drone's current latitude from DJI telemetry.
     * @param currentLon  Drone's current longitude from DJI telemetry.
     * @return true while still running.
     */
    fun tick(currentLat: Double, currentLon: Double): Boolean {
        if (!isRunning || isComplete) return false

        val seg = segments[currentSegIndex]
        val distToEnd = haversine(currentLat, currentLon, seg.end.latitude, seg.end.longitude)
        val advanceThreshold = max(3.0, seg.speedMs * 0.5)

        if (distToEnd <= advanceThreshold) {
            advanceSegment()
            return isRunning
        }

        val (vn, ve, vd) = seg.velocityNed()
        sendVelocity(vn, ve, vd, seg.yawDeg)

        progressLog.add(mapOf(
            "segment"       to currentSegIndex,
            "lat"           to currentLat,
            "lon"           to currentLon,
            "dist_to_end_m" to distToEnd,
            "speed_ms"      to seg.speedMs,
            "fly_backwards" to seg.flyBackwards,
            "yaw_deg"       to seg.yawDeg
        ))

        return true
    }

    // -----------------------------------------------------------------------
    // Internal
    // -----------------------------------------------------------------------

    private fun advanceSegment() {
        currentSegIndex++
        if (currentSegIndex >= segments.size) {
            finish()
        } else {
            val seg = segments[currentSegIndex]
            println("  → segment $currentSegIndex/${segments.size}: " +
                    "${seg.lengthM.toInt()} m @ ${seg.speedMs} m/s " +
                    "${if (seg.flyBackwards) "(BACKWARDS)" else ""}")
        }
    }

    private fun finish() {
        isRunning = false
        isComplete = true
        sendHover()

        if (route.updateHome) {
            route.finalWaypoint?.let { wp ->
                println("Updating home-point to ${wp.latitude}, ${wp.longitude}")
                onHomeUpdate?.invoke(wp.latitude, wp.longitude)
            }
        }
        println("Follow-mode route complete")
    }

    private fun sendVelocity(vn: Double, ve: Double, vd: Double, yaw: Double) {
        onVelocityCommand?.invoke(vn, ve, vd, yaw)
            ?: println("VEL vN=${"%.2f".format(vn)} vE=${"%.2f".format(ve)} vD=${"%.2f".format(vd)} yaw=${"%.1f".format(yaw)}°")
    }

    private fun sendHover() {
        val yaw = segments.getOrNull(
            currentSegIndex.coerceAtMost(segments.size - 1)
        )?.yawDeg ?: 0.0
        onVelocityCommand?.invoke(0.0, 0.0, 0.0, yaw)
            ?: println("VEL hover")
    }

    // -----------------------------------------------------------------------
    // Public controls
    // -----------------------------------------------------------------------

    /**
     * Update all remaining segment speeds to match the user's current pace.
     *
     * Call this from your wearable / phone sensor fusion loop whenever
     * the runner or rider changes speed.
     *
     * @param paceMs User's current ground speed in m/s.
     *               Typical values: walk 1.4, jog 3–4, run 5–6, cycle 8–12.
     */
    fun setPaceSpeed(paceMs: Double) {
        for (i in currentSegIndex until segments.size) {
            segments[i].speedMs = paceMs
        }
        println("Pace updated to ${"%.1f".format(paceMs)} m/s " +
                "(${"%.1f".format(paceMs * 3.6)} km/h) " +
                "for ${segments.size - currentSegIndex} remaining segments")
    }

    /**
     * Toggle backwards flight for all remaining segments.
     *
     * When enabled the drone reverses along the route so its front (camera)
     * faces the approaching athlete / rider.
     */
    fun setBackwardsFlight(enabled: Boolean) {
        for (i in currentSegIndex until segments.size) {
            segments[i].flyBackwards = enabled
        }
        println("Flight direction: ${if (enabled) "BACKWARDS" else "FORWARD"}")
    }

    fun exportProgressJson(): String = Gson().toJson(
        mapOf(
            "route_total_length_m" to route.totalLengthM,
            "segments_completed"   to currentSegIndex,
            "total_segments"       to segments.size,
            "route_complete"       to isComplete,
            "home_updated"         to (route.updateHome && isComplete),
            "log"                  to progressLog
        )
    )

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------

    private fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r = 6371000.0
        val phi1 = Math.toRadians(lat1); val phi2 = Math.toRadians(lat2)
        val dphi = Math.toRadians(lat2 - lat1)
        val dlam = Math.toRadians(lon2 - lon1)
        val a = sin(dphi / 2).pow(2) + cos(phi1) * cos(phi2) * sin(dlam / 2).pow(2)
        return r * 2 * asin(sqrt(a))
    }
}
