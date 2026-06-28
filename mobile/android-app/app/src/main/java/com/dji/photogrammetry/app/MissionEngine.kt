package com.dji.photogrammetry.app

import kotlin.math.*

// ---------------------------------------------------------------------------
// Drone profiles
// ---------------------------------------------------------------------------

data class DroneProfile(
    val id: String,
    val displayName: String,
    val focalLengthMm: Double,
    val sensorWidthMm: Double,
    val sensorHeightMm: Double,
    val imageWidthPx: Int,
    val imageHeightPx: Int,
    val maxSpeedMs: Double,
    val maxAltitudeM: Double,
    val hasRtk: Boolean,
    val requiresMsdkV5: Boolean
)

object DroneProfiles {
    val all = listOf(
        DroneProfile("mini3", "DJI Mini 3",
            6.7, 9.6, 7.2, 4032, 3024, 10.0, 120.0, false, true),
        DroneProfile("mini3_pro", "DJI Mini 3 Pro",
            6.7, 9.6, 7.2, 8064, 6048, 10.0, 120.0, false, true),
        DroneProfile("mini4_pro", "DJI Mini 4 Pro",
            6.7, 9.6, 7.2, 8064, 6048, 10.0, 120.0, false, true),
        DroneProfile("mavic3_classic", "DJI Mavic 3 Classic",
            12.29, 17.3, 13.0, 5280, 3956, 15.0, 120.0, false, true),
        DroneProfile("phantom4_rtk", "DJI Phantom 4 RTK",
            8.8, 13.2, 8.8, 5472, 3648, 15.0, 120.0, true, false),
        DroneProfile("generic", "Generic / Custom",
            24.0, 13.2, 8.8, 4000, 3000, 15.0, 120.0, false, false)
    )

    fun byId(id: String) = all.find { it.id == id } ?: all.first()
}

// ---------------------------------------------------------------------------
// Flight patterns
// ---------------------------------------------------------------------------

enum class FlightPattern(val label: String) {
    GRID("Grid"),
    DOUBLE_GRID("Double Grid (Cross-hatch)"),
    OBLIQUE("Oblique (45°)"),
    PERIMETER("Perimeter Orbit"),
    SPIRAL("Inward Spiral")
}

// ---------------------------------------------------------------------------
// Mission data model
// ---------------------------------------------------------------------------

data class LatLon(val lat: Double, val lon: Double)

data class RoiPolygon(val vertices: List<LatLon>) {
    val minLat get() = vertices.minOf { it.lat }
    val maxLat get() = vertices.maxOf { it.lat }
    val minLon get() = vertices.minOf { it.lon }
    val maxLon get() = vertices.maxOf { it.lon }
    val centreLat get() = (minLat + maxLat) / 2
    val centreLon get() = (minLon + maxLon) / 2
}

data class MissionWaypoint(
    val lat: Double,
    val lon: Double,
    val altM: Double,
    val headingDeg: Double,
    val gimbalPitchDeg: Double = -90.0,
    val shootPhoto: Boolean = true
)

data class ValidationResult(
    val isValid: Boolean,
    val errors: List<String>,
    val warnings: List<String>
)

data class Mission(
    val pattern: FlightPattern,
    val droneProfile: DroneProfile,
    val altitudeM: Double,
    val speedMs: Double,
    val frontOverlapPct: Double,
    val sideOverlapPct: Double,
    val waypoints: List<MissionWaypoint>,
    val gsdCm: Double,
    val coverageAreaHa: Double,
    val estimatedPhotos: Int,
    val photoDistanceM: Double,
    val photoIntervalS: Double
)

// ---------------------------------------------------------------------------
// Mission planner engine
// ---------------------------------------------------------------------------

object MissionEngine {

    private const val LAT_PER_M = 1.0 / 111_320.0

    private fun lonPerM(lat: Double) = 1.0 / (111_320.0 * cos(Math.toRadians(lat)))

    fun gsd(profile: DroneProfile, altM: Double): Double {
        val gsdM = (altM * profile.sensorWidthMm / 1000.0) /
                   (profile.focalLengthMm / 1000.0 * profile.imageWidthPx)
        return gsdM * 100.0 // cm/pixel
    }

    fun coverageDimensions(profile: DroneProfile, altM: Double): Pair<Double, Double> {
        val gsdM = gsd(profile, altM) / 100.0
        return Pair(gsdM * profile.imageWidthPx, gsdM * profile.imageHeightPx)
    }

    fun lineSpacing(profile: DroneProfile, altM: Double, sideOverlapPct: Double): Double {
        val (w, _) = coverageDimensions(profile, altM)
        return w * (1 - sideOverlapPct / 100.0)
    }

    fun photoInterval(profile: DroneProfile, altM: Double,
                      frontOverlapPct: Double, speedMs: Double): Pair<Double, Double> {
        val (_, h) = coverageDimensions(profile, altM)
        val dist = h * (1 - frontOverlapPct / 100.0)
        return Pair(dist, if (speedMs > 0) dist / speedMs else 0.0)
    }

    fun validate(profile: DroneProfile, altM: Double, speedMs: Double,
                 frontOverlapPct: Double, sideOverlapPct: Double): ValidationResult {
        val errors   = mutableListOf<String>()
        val warnings = mutableListOf<String>()

        if (speedMs > profile.maxSpeedMs)
            errors.add("Speed ${speedMs} m/s exceeds ${profile.displayName} max of ${profile.maxSpeedMs} m/s.")
        if (altM < 2.0)
            errors.add("Altitude ${altM} m is below the 2 m minimum.")
        if (altM > profile.maxAltitudeM)
            warnings.add("Altitude ${altM} m exceeds the recommended ${profile.maxAltitudeM} m AGL regulatory limit.")
        if (frontOverlapPct < 60 || sideOverlapPct < 50)
            warnings.add("Low overlap may reduce reconstruction quality. Recommended: ≥ 80 % front, ≥ 70 % side.")
        if (!profile.hasRtk)
            warnings.add("${profile.displayName} has no built-in RTK. Use GCPs for survey-grade accuracy.")
        if (profile.requiresMsdkV5)
            warnings.add("${profile.displayName} requires DJI Mobile SDK v5. MSDK v4 is incompatible.")

        return ValidationResult(errors.isEmpty(), errors, warnings)
    }

    fun plan(roi: RoiPolygon, pattern: FlightPattern, profile: DroneProfile,
             altM: Double, speedMs: Double,
             frontOverlapPct: Double, sideOverlapPct: Double): Mission {
        val waypoints = when (pattern) {
            FlightPattern.GRID        -> grid(roi, profile, altM, sideOverlapPct)
            FlightPattern.DOUBLE_GRID -> doubleGrid(roi, profile, altM, sideOverlapPct)
            FlightPattern.OBLIQUE     -> oblique(roi, altM)
            FlightPattern.PERIMETER   -> perimeter(roi, profile, altM, frontOverlapPct, speedMs)
            FlightPattern.SPIRAL      -> spiral(roi, profile, altM, sideOverlapPct)
        }
        val (photoDist, photoInterval) = photoInterval(profile, altM, frontOverlapPct, speedMs)
        val gsdCm = gsd(profile, altM)
        val widthM  = haversine(roi.minLat, roi.minLon, roi.minLat, roi.maxLon)
        val heightM = haversine(roi.minLat, roi.minLon, roi.maxLat, roi.minLon)
        val areaHa  = (widthM * heightM) / 10_000.0
        val estimatedPhotos = if (photoDist > 0) {
            var total = 0.0
            for (i in 0 until waypoints.size - 1) {
                total += haversine(waypoints[i].lat, waypoints[i].lon,
                                   waypoints[i+1].lat, waypoints[i+1].lon)
            }
            (total / photoDist).toInt()
        } else 0

        return Mission(pattern, profile, altM, speedMs, frontOverlapPct, sideOverlapPct,
                       waypoints, gsdCm, areaHa, estimatedPhotos, photoDist, photoInterval)
    }

    // ----- Patterns ---------------------------------------------------------

    private fun grid(roi: RoiPolygon, profile: DroneProfile,
                     altM: Double, sideOverlapPct: Double): List<MissionWaypoint> {
        val wps   = mutableListOf<MissionWaypoint>()
        val spacing = lineSpacing(profile, altM, sideOverlapPct)
        val midLat  = (roi.minLat + roi.maxLat) / 2
        var lat     = roi.minLat
        var dir     = 1
        while (lat <= roi.maxLat + 1e-8) {
            if (dir == 1) {
                wps.add(MissionWaypoint(lat, roi.minLon, altM, 90.0))
                wps.add(MissionWaypoint(lat, roi.maxLon, altM, 90.0))
            } else {
                wps.add(MissionWaypoint(lat, roi.maxLon, altM, 270.0))
                wps.add(MissionWaypoint(lat, roi.minLon, altM, 270.0))
            }
            lat += spacing * LAT_PER_M
            dir *= -1
        }
        return wps
    }

    private fun doubleGrid(roi: RoiPolygon, profile: DroneProfile,
                           altM: Double, sideOverlapPct: Double): List<MissionWaypoint> {
        val g1 = grid(roi, profile, altM, sideOverlapPct)
        val spacing = lineSpacing(profile, altM, sideOverlapPct)
        val midLat  = (roi.minLat + roi.maxLat) / 2
        val wps = mutableListOf<MissionWaypoint>()
        var lon = roi.minLon
        var dir = 1
        while (lon <= roi.maxLon + 1e-8) {
            if (dir == 1) {
                wps.add(MissionWaypoint(roi.minLat, lon, altM, 0.0))
                wps.add(MissionWaypoint(roi.maxLat, lon, altM, 0.0))
            } else {
                wps.add(MissionWaypoint(roi.maxLat, lon, altM, 180.0))
                wps.add(MissionWaypoint(roi.minLat, lon, altM, 180.0))
            }
            lon += spacing * lonPerM(midLat)
            dir *= -1
        }
        return g1 + wps
    }

    private fun oblique(roi: RoiPolygon, altM: Double): List<MissionWaypoint> {
        val wps = mutableListOf<MissionWaypoint>()
        for (v in roi.vertices) {
            var h = Math.toDegrees(atan2(roi.centreLon - v.lon, roi.centreLat - v.lat))
            if (h < 0) h += 360.0
            wps.add(MissionWaypoint(v.lat, v.lon, altM, h, -45.0))
        }
        if (wps.isNotEmpty()) wps.add(wps.first())
        return wps
    }

    private fun perimeter(roi: RoiPolygon, profile: DroneProfile,
                          altM: Double, frontOverlapPct: Double,
                          speedMs: Double): List<MissionWaypoint> {
        val (photoDist, _) = photoInterval(profile, altM, frontOverlapPct, speedMs)
        val dense = densify(roi.vertices, photoDist)
        val wps = mutableListOf<MissionWaypoint>()
        for (p in dense) {
            var h = Math.toDegrees(atan2(roi.centreLon - p.lon, roi.centreLat - p.lat))
            if (h < 0) h += 360.0
            wps.add(MissionWaypoint(p.lat, p.lon, altM, h, -45.0))
        }
        if (wps.isNotEmpty()) wps.add(wps.first())
        return wps
    }

    private fun spiral(roi: RoiPolygon, profile: DroneProfile,
                       altM: Double, sideOverlapPct: Double): List<MissionWaypoint> {
        val wps     = mutableListOf<MissionWaypoint>()
        val spacing = lineSpacing(profile, altM, sideOverlapPct)
        val rMax    = minOf(
            (roi.maxLat - roi.minLat) / (2 * LAT_PER_M),
            (roi.maxLon - roi.minLon) / (2 * lonPerM(roi.centreLat))
        )
        if (rMax < spacing) return wps
        val rings = (rMax / spacing).toInt()
        val steps = 36
        for (ring in 0..rings) {
            val r = rMax - ring * spacing
            if (r <= 0) break
            for (s in 0 until steps) {
                val a = Math.toRadians(s.toDouble() / steps * 360.0)
                wps.add(MissionWaypoint(
                    roi.centreLat + cos(a) * r * LAT_PER_M,
                    roi.centreLon + sin(a) * r * lonPerM(roi.centreLat),
                    altM,
                    (Math.toDegrees(a) + 90) % 360
                ))
            }
        }
        wps.add(MissionWaypoint(roi.centreLat, roi.centreLon, altM, 0.0))
        return wps
    }

    // ----- Helpers ----------------------------------------------------------

    private fun densify(vertices: List<LatLon>, maxSegM: Double): List<LatLon> {
        if (vertices.isEmpty()) return emptyList()
        val out = mutableListOf<LatLon>()
        val n = vertices.size
        for (i in vertices.indices) {
            val a = vertices[i]; val b = vertices[(i + 1) % n]
            val edgeM = haversine(a.lat, a.lon, b.lat, b.lon)
            val steps = maxOf(1, ceil(edgeM / maxSegM).toInt())
            for (s in 0 until steps) {
                val t = s.toDouble() / steps
                out.add(LatLon(a.lat + t * (b.lat - a.lat), a.lon + t * (b.lon - a.lon)))
            }
        }
        return out
    }

    fun haversine(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
        val r    = 6_371_000.0
        val phi1 = Math.toRadians(lat1); val phi2 = Math.toRadians(lat2)
        val dphi = Math.toRadians(lat2 - lat1)
        val dlam = Math.toRadians(lon2 - lon1)
        val a    = sin(dphi / 2).pow(2) + cos(phi1) * cos(phi2) * sin(dlam / 2).pow(2)
        return r * 2 * asin(sqrt(a))
    }

    fun missionToJson(mission: Mission, roiVertices: List<LatLon>): String {
        val sb = StringBuilder()
        sb.appendLine("{")
        sb.appendLine("  \"sdk_version\": \"0.2.0\",")
        sb.appendLine("  \"drone\": \"${mission.droneProfile.id}\",")
        sb.appendLine("  \"pattern\": \"${mission.pattern.name.lowercase()}\",")
        sb.appendLine("  \"altitude_m\": ${mission.altitudeM},")
        sb.appendLine("  \"speed_ms\": ${mission.speedMs},")
        sb.appendLine("  \"front_overlap_pct\": ${mission.frontOverlapPct},")
        sb.appendLine("  \"side_overlap_pct\": ${mission.sideOverlapPct},")
        sb.appendLine("  \"gsd_cm_per_pixel\": ${"%.2f".format(mission.gsdCm)},")
        sb.appendLine("  \"coverage_area_ha\": ${"%.4f".format(mission.coverageAreaHa)},")
        sb.appendLine("  \"estimated_photos\": ${mission.estimatedPhotos},")
        sb.appendLine("  \"photo_trigger_distance_m\": ${"%.2f".format(mission.photoDistanceM)},")
        sb.appendLine("  \"photo_trigger_interval_s\": ${"%.2f".format(mission.photoIntervalS)},")
        sb.appendLine("  \"roi\": [")
        roiVertices.forEachIndexed { i, v ->
            val comma = if (i < roiVertices.size - 1) "," else ""
            sb.appendLine("    {\"lat\": ${v.lat}, \"lon\": ${v.lon}}$comma")
        }
        sb.appendLine("  ],")
        sb.appendLine("  \"waypoints\": [")
        mission.waypoints.forEachIndexed { i, wp ->
            val comma = if (i < mission.waypoints.size - 1) "," else ""
            sb.appendLine("    {\"lat\": ${wp.lat}, \"lon\": ${wp.lon}, " +
                "\"alt\": ${wp.altM}, \"heading\": ${wp.headingDeg}, " +
                "\"gimbal\": ${wp.gimbalPitchDeg}, \"shoot_photo\": ${wp.shootPhoto}}$comma")
        }
        sb.appendLine("  ]")
        sb.append("}")
        return sb.toString()
    }

    fun missionToKml(mission: Mission, roiVertices: List<LatLon>): String {
        val sb = StringBuilder()
        sb.appendLine("""<?xml version="1.0" encoding="UTF-8"?>""")
        sb.appendLine("""<kml xmlns="http://www.opengis.net/kml/2.2">""")
        sb.appendLine("""  <Document>""")
        sb.appendLine("""    <name>DJI Photogrammetry Mission</name>""")
        sb.appendLine("""    <description>Pattern: ${mission.pattern.label} | Drone: ${mission.droneProfile.displayName} | Alt: ${mission.altitudeM}m | GSD: ${"%.2f".format(mission.gsdCm)}cm/px</description>""")
        // ROI
        sb.appendLine("""    <Placemark><name>ROI</name><LineString><coordinates>""")
        roiVertices.forEach { v -> sb.append("${v.lon},${v.lat},0 ") }
        if (roiVertices.isNotEmpty()) sb.append("${roiVertices.first().lon},${roiVertices.first().lat},0")
        sb.appendLine("""    </coordinates></LineString></Placemark>""")
        // Flight path
        sb.appendLine("""    <Placemark><name>Flight Path</name><LineString><coordinates>""")
        mission.waypoints.forEach { wp -> sb.append("${wp.lon},${wp.lat},${wp.altM} ") }
        sb.appendLine("""    </coordinates></LineString></Placemark>""")
        // Waypoints
        mission.waypoints.forEachIndexed { i, wp ->
            sb.appendLine("""    <Placemark><name>WP${i+1}</name><Point><coordinates>${wp.lon},${wp.lat},${wp.altM}</coordinates></Point></Placemark>""")
        }
        sb.appendLine("""  </Document>""")
        sb.appendLine("""</kml>""")
        return sb.toString()
    }
}
