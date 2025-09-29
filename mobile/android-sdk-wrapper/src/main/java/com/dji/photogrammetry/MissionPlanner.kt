package com.dji.photogrammetry

import com.google.gson.Gson
import com.google.gson.annotations.SerializedName
import kotlin.math.*

/**
 * Mission planner for DJI drone photogrammetry surveys.
 * 
 * Generates flight patterns (grid, double-grid, oblique) with configurable
 * altitude, overlap percentages, camera settings, and ROI polygons.
 */

enum class FlightPattern {
    @SerializedName("grid")
    GRID,
    @SerializedName("double_grid") 
    DOUBLE_GRID,
    @SerializedName("oblique")
    OBLIQUE
}

data class CameraSettings(
    val focalLengthMm: Double = 24.0,
    val sensorWidthMm: Double = 13.2,
    val sensorHeightMm: Double = 8.8,
    val imageWidthPx: Int = 4000,
    val imageHeightPx: Int = 3000,
    val gimbalPitchDeg: Double = -90.0
)

data class MissionSettings(
    val altitudeM: Double = 100.0,
    val frontOverlapPct: Double = 80.0,
    val sideOverlapPct: Double = 70.0,
    val flightSpeedMs: Double = 15.0,
    val triggerDistanceM: Double? = null,
    val triggerTimeS: Double? = null
)

data class Waypoint(
    val latitude: Double,
    val longitude: Double,
    val altitudeM: Double,
    val headingDeg: Double = 0.0,
    val gimbalPitchDeg: Double = -90.0,
    val triggerCamera: Boolean = true
)

data class ROIPolygon(
    val vertices: List<Pair<Double, Double>>
) {
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
    val minLat: Double,
    val minLon: Double,
    val maxLat: Double,
    val maxLon: Double
)

data class PhotoIntervals(
    val distanceM: Double,
    val timeS: Double
)

data class MissionStats(
    val totalWaypoints: Int,
    val gsdCmPerPixel: Double,
    val coverageAreaM2: Double,
    val estimatedPhotos: Int
)

data class Mission(
    val pattern: String,
    val waypoints: List<Waypoint>,
    val settings: MissionSettings,
    val cameraSettings: CameraSettings,
    val photoIntervals: PhotoIntervals,
    val missionStats: MissionStats
)

class MissionPlanner(private val cameraSettings: CameraSettings = CameraSettings()) {

    /**
     * Calculate Ground Sampling Distance (GSD) for given altitude.
     */
    fun calculateGroundSamplingDistance(altitudeM: Double): Double {
        val gsdM = (altitudeM * (cameraSettings.sensorWidthMm / 1000.0)) / 
                   ((cameraSettings.focalLengthMm / 1000.0) * cameraSettings.imageWidthPx)
        return gsdM * 100.0 // Convert to cm/pixel
    }

    /**
     * Calculate ground coverage dimensions for single image.
     */
    fun calculateCoverageDimensions(altitudeM: Double): Pair<Double, Double> {
        val gsdM = calculateGroundSamplingDistance(altitudeM) / 100.0 // cm to m
        val widthM = gsdM * cameraSettings.imageWidthPx
        val heightM = gsdM * cameraSettings.imageHeightPx
        return Pair(widthM, heightM)
    }

    /**
     * Calculate spacing between parallel flight lines.
     */
    fun calculateFlightLineSpacing(altitudeM: Double, sideOverlapPct: Double): Double {
        val (coverageWidth, _) = calculateCoverageDimensions(altitudeM)
        return coverageWidth * (1 - sideOverlapPct / 100.0)
    }

    /**
     * Calculate photo capture interval for distance and time-based triggers.
     */
    fun calculatePhotoInterval(
        altitudeM: Double, 
        frontOverlapPct: Double, 
        flightSpeedMs: Double
    ): PhotoIntervals {
        val (_, coverageHeight) = calculateCoverageDimensions(altitudeM)
        val effectiveDistance = coverageHeight * (1 - frontOverlapPct / 100.0)
        val timeInterval = effectiveDistance / flightSpeedMs
        return PhotoIntervals(effectiveDistance, timeInterval)
    }

    /**
     * Generate grid pattern mission waypoints.
     */
    fun generateGridMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints = mutableListOf<Waypoint>()
        val bounds = roi.getBounds()
        
        // Calculate flight line spacing
        val lineSpacing = calculateFlightLineSpacing(settings.altitudeM, settings.sideOverlapPct)
        
        // Convert to approximate meters (rough approximation for mission planning)
        val latPerM = 1.0 / 111320.0 // degrees per meter latitude
        val lonPerM = 1.0 / (111320.0 * cos(Math.toRadians((bounds.minLat + bounds.maxLat) / 2)))
        
        // Generate parallel flight lines
        var currentLat = bounds.minLat
        var lineDirection = 1 // 1 for west->east, -1 for east->west
        
        while (currentLat <= bounds.maxLat) {
            if (lineDirection == 1) {
                // West to East
                waypoints.add(Waypoint(
                    latitude = currentLat,
                    longitude = bounds.minLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 90.0, // East
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
                waypoints.add(Waypoint(
                    latitude = currentLat,
                    longitude = bounds.maxLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 90.0,
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
            } else {
                // East to West
                waypoints.add(Waypoint(
                    latitude = currentLat,
                    longitude = bounds.maxLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 270.0, // West
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
                waypoints.add(Waypoint(
                    latitude = currentLat,
                    longitude = bounds.minLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 270.0,
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
            }
            
            // Move to next line
            currentLat += lineSpacing * latPerM
            lineDirection *= -1 // Alternate direction
        }
        
        return waypoints
    }

    /**
     * Generate double grid (cross-hatch) pattern mission.
     */
    fun generateDoubleGridMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        // First grid (North-South lines)
        val grid1Waypoints = generateGridMission(roi, settings)
        
        // Second grid (East-West lines) - rotate the coordinates
        val bounds = roi.getBounds()
        val lineSpacing = calculateFlightLineSpacing(settings.altitudeM, settings.sideOverlapPct)
        
        val latPerM = 1.0 / 111320.0
        val lonPerM = 1.0 / (111320.0 * cos(Math.toRadians((bounds.minLat + bounds.maxLat) / 2)))
        
        val grid2Waypoints = mutableListOf<Waypoint>()
        var currentLon = bounds.minLon
        var lineDirection = 1
        
        while (currentLon <= bounds.maxLon) {
            if (lineDirection == 1) {
                // South to North
                grid2Waypoints.add(Waypoint(
                    latitude = bounds.minLat,
                    longitude = currentLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 0.0, // North
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
                grid2Waypoints.add(Waypoint(
                    latitude = bounds.maxLat,
                    longitude = currentLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 0.0,
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
            } else {
                // North to South
                grid2Waypoints.add(Waypoint(
                    latitude = bounds.maxLat,
                    longitude = currentLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 180.0, // South
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
                grid2Waypoints.add(Waypoint(
                    latitude = bounds.minLat,
                    longitude = currentLon,
                    altitudeM = settings.altitudeM,
                    headingDeg = 180.0,
                    gimbalPitchDeg = cameraSettings.gimbalPitchDeg
                ))
            }
            
            currentLon += lineSpacing * lonPerM
            lineDirection *= -1
        }
        
        return grid1Waypoints + grid2Waypoints
    }

    /**
     * Generate oblique imagery mission with angled camera.
     */
    fun generateObliqueMission(roi: ROIPolygon, settings: MissionSettings): List<Waypoint> {
        val waypoints = mutableListOf<Waypoint>()
        
        // Calculate center point of ROI
        val bounds = roi.getBounds()
        val centerLat = (bounds.minLat + bounds.maxLat) / 2
        val centerLon = (bounds.minLon + bounds.maxLon) / 2
        
        // Create waypoints around perimeter
        for (vertex in roi.vertices) {
            val (lat, lon) = vertex
            
            // Calculate heading toward center
            val dlat = centerLat - lat
            val dlon = centerLon - lon
            var headingDeg = Math.toDegrees(atan2(dlon, dlat))
            if (headingDeg < 0) {
                headingDeg += 360
            }
            
            waypoints.add(Waypoint(
                latitude = lat,
                longitude = lon,
                altitudeM = settings.altitudeM,
                headingDeg = headingDeg,
                gimbalPitchDeg = -45.0 // 45-degree oblique angle
            ))
        }
        
        // Close the loop by returning to first waypoint
        if (waypoints.isNotEmpty()) {
            waypoints.add(waypoints[0])
        }
        
        return waypoints
    }

    /**
     * Create complete mission with specified pattern and settings.
     */
    fun createMission(
        pattern: FlightPattern,
        roi: ROIPolygon,
        settings: MissionSettings
    ): Mission {
        val waypoints = when (pattern) {
            FlightPattern.GRID -> generateGridMission(roi, settings)
            FlightPattern.DOUBLE_GRID -> generateDoubleGridMission(roi, settings)
            FlightPattern.OBLIQUE -> generateObliqueMission(roi, settings)
        }
        
        // Calculate photo intervals
        val photoIntervals = calculatePhotoInterval(
            settings.altitudeM, 
            settings.frontOverlapPct, 
            settings.flightSpeedMs
        )
        
        // Calculate mission statistics
        val gsd = calculateGroundSamplingDistance(settings.altitudeM)
        val coverageArea = calculateRoiArea(roi)
        val estimatedPhotos = estimatePhotoCount(waypoints, photoIntervals.distanceM)
        
        val missionStats = MissionStats(
            totalWaypoints = waypoints.size,
            gsdCmPerPixel = gsd,
            coverageAreaM2 = coverageArea,
            estimatedPhotos = estimatedPhotos
        )
        
        return Mission(
            pattern = pattern.name.lowercase(),
            waypoints = waypoints,
            settings = settings,
            cameraSettings = cameraSettings,
            photoIntervals = photoIntervals,
            missionStats = missionStats
        )
    }

    private fun calculateRoiArea(roi: ROIPolygon): Double {
        // Simplified area calculation using bounding box
        val bounds = roi.getBounds()
        
        // Convert to meters (rough approximation)
        val latDiffM = (bounds.maxLat - bounds.minLat) * 111320.0
        val lonDiffM = (bounds.maxLon - bounds.minLon) * 111320.0 * 
                      cos(Math.toRadians((bounds.minLat + bounds.maxLat) / 2))
        
        return latDiffM * lonDiffM
    }

    private fun estimatePhotoCount(waypoints: List<Waypoint>, distanceInterval: Double): Int {
        var totalDistance = 0.0
        for (i in 0 until waypoints.size - 1) {
            val wp1 = waypoints[i]
            val wp2 = waypoints[i + 1]
            
            // Simplified distance calculation
            val dlat = (wp2.latitude - wp1.latitude) * 111320.0
            val dlon = (wp2.longitude - wp1.longitude) * 111320.0 * 
                      cos(Math.toRadians(wp1.latitude))
            val distance = sqrt(dlat * dlat + dlon * dlon)
            totalDistance += distance
        }
        
        return if (distanceInterval > 0) (totalDistance / distanceInterval).toInt() else 0
    }

    /**
     * Export mission to JSON string.
     */
    fun exportMissionToJson(mission: Mission): String {
        return Gson().toJson(mission)
    }

    /**
     * Import mission from JSON string.
     */
    fun importMissionFromJson(json: String): Mission {
        return Gson().fromJson(json, Mission::class.java)
    }
}