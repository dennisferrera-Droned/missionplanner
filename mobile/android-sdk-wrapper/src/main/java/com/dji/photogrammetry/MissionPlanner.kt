package com.dji.photogrammetry

import com.google.gson.Gson
import com.google.gson.annotations.SerializedName
import kotlin.math.*

/**
 * Mission planner for DJI drone photogrammetry surveys.
 *
 * Compatible with DJI Mobile SDK v5 (required for Mini 3, Mini 3 Pro,
 * Mini 4 Pro, Mavic 3 series). Also backwards-compatible with MSDK v4
 * drones (Phantom 4 RTK, Mavic 2 Pro, etc.) when the app targets v4.
 *
 * Usage — Mini 3 quick-start
 * --------------------------
 *   val profile  = DroneProfiles.MINI_3
 *   val planner  = MissionPlanner(profile)
 *   val roi      = ROIPolygon(listOf(Pair(47.37, 8.54), ...))
 *   val settings = MissionSettings(altitudeM = 80.0, flightSpeedMs = 8.0)
 *
 *   val validation = DroneProfiles.validate(profile, settings)
 *   if (!validation.isValid) { /* handle errors */ }
 *
 *   val mission = planner.createMission(FlightPattern.GRID, roi, settings)
 *   val json    = planner.exportMissionToJson(mission)
 */

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

enum class FlightPattern {
    @SerializedName("grid")          GRID,
    @SerializedName("double_grid")   DOUBLE_GRID,
    @SerializedName("oblique")       OBLIQUE,
    @SerializedName("perimeter")     PERIMETER,
    @SerializedName("spiral")        SPIRAL,
    @SerializedName("terrain_follow") TERRAIN_FOLLOW
}

// ---------------------------------------------------------------------------
// Data classes
// ---------------------------------------------------------------------------

data class CameraSettings(
    val focalLengthMm:  Double = 6.7,
    val sensorWidthMm:  Double = 9.6,
    val sensorHeightMm: Double = 7.2,
    val imageWidthPx:   Int    = 4032,
    val imageHeightPx:  Int    = 3024,
    val gimbalPitchDeg: Double = -90.0
)

data class MissionSettings(
    val altitudeM:      Double  = 80.0,     // Mini 3 safe default: 80 m AGL
    val frontOverlapPct: Double = 80.0,
    val sideOverlapPct:  Double = 70.0,
    val flightSpeedMs:  Double  = 8.0,      // Mini 3 recommended: 8 m/s for photo
    val triggerDistanceM: Double? = null,
    val triggerTimeS:    Double? = null
)

data class Waypoint(
    val latitude:       Double,
    val longitude:      Double,
    val altitudeM:      Double,
    val headingDeg:     Double  = 0.0,
    val gimbalPitchDeg: Double  = -90.0,
    val triggerCamera:  Boolean = true,
    val flyBackwards:   Boolean = false
)

data class ROIPolygon(val vertices: List<Pair<Double, Double>>) {
    fun getBounds(): Bounds {
        val lats = vertices.map { it.first }
        val lons = vertices.map { it.second }
        return Bounds(
            minLat = lats.minOrNull() ?: 0.0,
            minLon = lons.minOrNull() ?: 0.0,
            maxLat = lats.maxOrNull() ?: 0.0,
            maxLon = lons.maxOrNull() ?: 0.0
        )
    }
}

data class Bounds(
    val minLat: Double, val minLon: Double,
    val maxLat: Double, val maxLon: Double
)

data class PhotoIntervals(val distanceM: Double, val timeS: Double)

data class MissionStats(
    val totalWaypoints:    Int,
    val gsdCmPerPixel:     Double,
    val coverageAreaM2:    Double,
    val estimatedPhotos:   Int,
    val droneModel:        String = "Generic",
    val msdkVersion:       Int    = 5
)

data class Mission(
    val pattern:        String,
    val waypoints:      List<Waypoint>,
    val settings:       MissionSettings,
    val cameraSettings: CameraSettings,
    val photoIntervals: PhotoIntervals,
    val missionStats:   MissionStats
)

// ---------------------------------------------------------------------------
// MissionPlanner
// ---------------------------------------------------------------------------

class MissionPlanner(
    private val profile: DroneProfile = DroneProfiles.MINI_3
) {
    private val cameraSettings get() = profile.cameraSettings
    private val limits         get() = profile.limits

    // ------------------------------------------------------------------
    // Calculations
    // ------------------------------------------------------------------

    fun calculateGroundSamplingDistance(altitudeM: Double): Double {
        val gsdM = (altitudeM * (cameraSettings.sensorWidthMm / 1000.0)) /
                   ((cameraSettings.focalLengthMm / 1000.0) * cameraSettings.imageWidthPx)
        return gsdM * 100.0
    }

    fun calculateCoverageDimensions(altitudeM: Double): Pair<Double, Double> {
        val gsdM = calculateGroundSamplingDistance(altitudeM) / 100.0
        return Pair(gsdM * cameraSettings.imageWidthPx, gsdM * cameraSettings.imageHeightPx)
    }

    fun calculateFlightLineSpacing(altitudeM: Double, sideOverlapPct: Double): Double {
        val (coverageWidth, _) = calculateCoverageDimensions(altitudeM)
        return coverageWidth * (1 - sideOverlapPct / 100.0)
    }

    fun calculatePhotoInterval(
        altitudeM: Double, frontOverlapPct: Double, flightSpeedMs: Double
    ): PhotoIntervals {
        val (_, coverageHeight) = calculateCoverageDimensions(altitudeM)
        val effectiveDistance   = coverageHeight * (1 - frontOverlapPct / 100.0)
        return PhotoIntervals(effectiveDistance, effectiveDistance / flightSpeedMs)
    }

    // ------------------------------------------------------------------
    // Validation
    // ------------------------------------------------------------------

    /**
     * Validate mission settings against this drone's hard limits.
     *
     * Always call this before createMission() and surface any errors to the
     * user — the firmware will reject out-of-range values silently.
     */
    fun validate(settings: MissionSettings): ValidationResult =
        DroneProfiles.validate(profile, settings)

    // ------------------------------------------------------------------
    // Pattern generators
    // ------------------------------------------------------------------

    fun generateGridMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints    = mutableListOf<Waypoint>()
        val bounds       = roi.getBounds()
        val lineSpacing  = calculateFlightLineSpacing(settings.altitudeM, settings.sideOverlapPct)
        val latPerM      = 1.0 / 111320.0
        val midLat       = (bounds.minLat + bounds.maxLat) / 2
        val lonPerM      = 1.0 / (111320.0 * cos(Math.toRadians(midLat)))

        var currentLat   = bounds.minLat
        var dir          = 1

        while (currentLat <= bounds.maxLat) {
            if (dir == 1) {
                waypoints.add(wp(currentLat, bounds.minLon, settings.altitudeM, 90.0))
                waypoints.add(wp(currentLat, bounds.maxLon, settings.altitudeM, 90.0))
            } else {
                waypoints.add(wp(currentLat, bounds.maxLon, settings.altitudeM, 270.0))
                waypoints.add(wp(currentLat, bounds.minLon, settings.altitudeM, 270.0))
            }
            currentLat += lineSpacing * latPerM
            dir *= -1
        }
        return waypoints
    }

    fun generateDoubleGridMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val grid1 = generateGridMission(roi, settings)
        val bounds = roi.getBounds()
        val lineSpacing = calculateFlightLineSpacing(settings.altitudeM, settings.sideOverlapPct)
        val lonPerM = 1.0 / (111320.0 * cos(Math.toRadians((bounds.minLat + bounds.maxLat) / 2)))
        val grid2 = mutableListOf<Waypoint>()
        var currentLon = bounds.minLon
        var dir = 1

        while (currentLon <= bounds.maxLon) {
            if (dir == 1) {
                grid2.add(wp(bounds.minLat, currentLon, settings.altitudeM, 0.0))
                grid2.add(wp(bounds.maxLat, currentLon, settings.altitudeM, 0.0))
            } else {
                grid2.add(wp(bounds.maxLat, currentLon, settings.altitudeM, 180.0))
                grid2.add(wp(bounds.minLat, currentLon, settings.altitudeM, 180.0))
            }
            currentLon += lineSpacing * lonPerM
            dir *= -1
        }
        return grid1 + grid2
    }

    fun generateObliqueMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints = mutableListOf<Waypoint>()
        val bounds    = roi.getBounds()
        val cLat = (bounds.minLat + bounds.maxLat) / 2
        val cLon = (bounds.minLon + bounds.maxLon) / 2

        for ((lat, lon) in roi.vertices) {
            var heading = Math.toDegrees(atan2(cLon - lon, cLat - lat))
            if (heading < 0) heading += 360.0
            waypoints.add(Waypoint(lat, lon, settings.altitudeM, heading, -45.0))
        }
        if (waypoints.isNotEmpty()) waypoints.add(waypoints[0])
        return waypoints
    }

    fun generatePerimeterMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints  = mutableListOf<Waypoint>()
        val bounds     = roi.getBounds()
        val cLat       = (bounds.minLat + bounds.maxLat) / 2
        val cLon       = (bounds.minLon + bounds.maxLon) / 2
        val interval   = calculatePhotoInterval(
            settings.altitudeM, settings.frontOverlapPct, settings.flightSpeedMs
        ).distanceM
        val dense      = densifyPolygon(roi.vertices, interval)

        for ((lat, lon) in dense) {
            var heading = Math.toDegrees(atan2(cLon - lon, cLat - lat))
            if (heading < 0) heading += 360.0
            waypoints.add(Waypoint(lat, lon, settings.altitudeM, heading, -45.0))
        }
        if (waypoints.isNotEmpty()) waypoints.add(waypoints[0])
        return waypoints
    }

    fun generateSpiralMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints    = mutableListOf<Waypoint>()
        val bounds       = roi.getBounds()
        val cLat         = (bounds.minLat + bounds.maxLat) / 2
        val cLon         = (bounds.minLon + bounds.maxLon) / 2
        val latPerM      = 1.0 / 111320.0
        val lonPerM      = 1.0 / (111320.0 * cos(Math.toRadians(cLat)))
        val ringSpacing  = calculateFlightLineSpacing(settings.altitudeM, settings.sideOverlapPct)
        val maxRadiusM   = minOf(
            (bounds.maxLat - bounds.minLat) / (2 * latPerM),
            (bounds.maxLon - bounds.minLon) / (2 * lonPerM)
        )
        if (maxRadiusM < ringSpacing) return waypoints

        val numRings = (maxRadiusM / ringSpacing).toInt()
        val steps    = 36

        for (ring in 0..numRings) {
            val radiusM = maxRadiusM - ring * ringSpacing
            if (radiusM <= 0) break
            for (step in 0 until steps) {
                val angleDeg = (step.toDouble() / steps) * 360.0
                val rad      = Math.toRadians(angleDeg)
                val lat      = cLat + cos(rad) * radiusM * latPerM
                val lon      = cLon + sin(rad) * radiusM * lonPerM
                waypoints.add(wp(lat, lon, settings.altitudeM, (angleDeg + 90) % 360))
            }
        }
        waypoints.add(wp(cLat, cLon, settings.altitudeM, 0.0))
        return waypoints
    }

    // ------------------------------------------------------------------
    // create-mission factory
    // ------------------------------------------------------------------

    fun createMission(
        pattern: FlightPattern,
        roi: ROIPolygon,
        settings: MissionSettings
    ): Mission {
        val waypoints = when (pattern) {
            FlightPattern.GRID          -> generateGridMission(roi, settings)
            FlightPattern.DOUBLE_GRID   -> generateDoubleGridMission(roi, settings)
            FlightPattern.OBLIQUE       -> generateObliqueMission(roi, settings)
            FlightPattern.PERIMETER     -> generatePerimeterMission(roi, settings)
            FlightPattern.SPIRAL        -> generateSpiralMission(roi, settings)
            FlightPattern.TERRAIN_FOLLOW -> generateGridMission(roi, settings) // DEM needed at runtime
        }

        val photoIntervals = calculatePhotoInterval(
            settings.altitudeM, settings.frontOverlapPct, settings.flightSpeedMs
        )

        return Mission(
            pattern        = pattern.name.lowercase(),
            waypoints      = waypoints,
            settings       = settings,
            cameraSettings = cameraSettings,
            photoIntervals = photoIntervals,
            missionStats   = MissionStats(
                totalWaypoints  = waypoints.size,
                gsdCmPerPixel   = calculateGroundSamplingDistance(settings.altitudeM),
                coverageAreaM2  = calculateRoiArea(roi),
                estimatedPhotos = estimatePhotoCount(waypoints, photoIntervals.distanceM),
                droneModel      = profile.displayName,
                msdkVersion     = if (limits.requiresMsdkV5) 5 else 4
            )
        )
    }

    // ------------------------------------------------------------------
    // MSDK v5 upload helper (scaffolding — wire to real SDK in your app)
    // ------------------------------------------------------------------

    /**
     * Convert this SDK's Mission to a MSDK v5 WaypointV2MissionOperator upload.
     *
     * In your app, replace the body with:
     *
     *   val operator = WaypointV2MissionOperator.getInstance()
     *   operator.uploadMission(waypointMission, callback)
     *
     * MSDK v5 imports (add to your Activity/Fragment):
     *   import dji.v5.manager.aircraft.waypoint3.WaypointMissionManager
     *   import dji.v5.common.callback.CommonCallbacks
     *
     * This method is provided as a documented integration point so the
     * application layer knows exactly which MSDK v5 APIs to call.
     */
    fun buildMsdkV5WaypointList(mission: Mission): List<Map<String, Any>> {
        return mission.waypoints.mapIndexed { index, wp ->
            mapOf(
                "index"             to index,
                "coordinate"        to mapOf("latitude" to wp.latitude, "longitude" to wp.longitude),
                "altitude"          to wp.altitudeM,
                "heading"           to wp.headingDeg,
                "gimbalPitch"       to wp.gimbalPitchDeg,
                "shootPhoto"        to wp.triggerCamera,
                "turnMode"          to "CLOCK_WISE",
                "waypointActions"   to if (wp.triggerCamera)
                    listOf(mapOf("actionType" to "SHOOT_PHOTO")) else emptyList<Any>()
            )
        }
    }

    // ------------------------------------------------------------------
    // JSON I/O
    // ------------------------------------------------------------------

    fun exportMissionToJson(mission: Mission): String = Gson().toJson(mission)
    fun importMissionFromJson(json: String): Mission  = Gson().fromJson(json, Mission::class.java)

    // ------------------------------------------------------------------
    // Private helpers
    // ------------------------------------------------------------------

    private fun wp(lat: Double, lon: Double, alt: Double, heading: Double,
                   gimbal: Double = cameraSettings.gimbalPitchDeg) =
        Waypoint(lat, lon, alt, heading, gimbal)

    private fun densifyPolygon(
        vertices: List<Pair<Double, Double>>,
        maxSegmentM: Double
    ): List<Pair<Double, Double>> {
        if (vertices.isEmpty()) return emptyList()
        val dense = mutableListOf<Pair<Double, Double>>()
        val n = vertices.size
        for (i in vertices.indices) {
            val (lat1, lon1) = vertices[i]
            val (lat2, lon2) = vertices[(i + 1) % n]
            val edgeM = haversine(lat1, lon1, lat2, lon2)
            val steps = maxOf(1, ceil(edgeM / maxSegmentM).toInt())
            for (step in 0 until steps) {
                val t = step.toDouble() / steps
                dense.add(Pair(lat1 + t * (lat2 - lat1), lon1 + t * (lon2 - lon1)))
            }
        }
        return dense
    }

    private fun calculateRoiArea(roi: ROIPolygon): Double {
        val b = roi.getBounds()
        val latM = (b.maxLat - b.minLat) * 111320.0
        val lonM = (b.maxLon - b.minLon) * 111320.0 *
                   cos(Math.toRadians((b.minLat + b.maxLat) / 2))
        return latM * lonM
    }

    private fun estimatePhotoCount(waypoints: List<Waypoint>, distanceInterval: Double): Int {
        var total = 0.0
        for (i in 0 until waypoints.size - 1) {
            val w1 = waypoints[i]; val w2 = waypoints[i + 1]
            val dlat = (w2.latitude  - w1.latitude)  * 111320.0
            val dlon = (w2.longitude - w1.longitude) * 111320.0 * cos(Math.toRadians(w1.latitude))
            total += sqrt(dlat * dlat + dlon * dlon)
        }
        return if (distanceInterval > 0) (total / distanceInterval).toInt() else 0
    }

    private fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r    = 6371000.0
        val phi1 = Math.toRadians(lat1); val phi2 = Math.toRadians(lat2)
        val dphi = Math.toRadians(lat2 - lat1)
        val dlam = Math.toRadians(lon2 - lon1)
        val a    = sin(dphi / 2).pow(2) + cos(phi1) * cos(phi2) * sin(dlam / 2).pow(2)
        return r * 2 * asin(sqrt(a))
    }
}
