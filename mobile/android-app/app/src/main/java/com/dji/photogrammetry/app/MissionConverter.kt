package com.dji.photogrammetry.app

import com.google.gson.Gson
import com.google.gson.JsonObject
import com.google.gson.JsonArray
import dji.v5.manager.aircraft.waypoint3.model.WaypointMission

/**
 * Converts the app's own JSON mission format into a DJI [WaypointMission].
 *
 * Supported JSON shapes
 * ─────────────────────
 * 1. Mission Planner output (from ExportActivity / MissionEngine.missionToJson):
 *    {
 *      "type": "grid_mission",
 *      "speed_ms": 8.0,
 *      "waypoints": [ {"lat": …, "lon": …, "alt_m": …}, … ]
 *    }
 *
 * 2. Route Follow output (from RouteFollowActivity.buildRouteJson):
 *    {
 *      "type": "follow_route",
 *      "default_speed_ms": 5,
 *      "waypoints": [ {"name": …, "lat": …, "lon": …, "alt_m": …}, … ]
 *    }
 */
object MissionConverter {

    private val gson = Gson()

    /**
     * Parse [json] and return a [WaypointMission] ready to pass to
     * [DjiSdkManager.uploadMission].
     *
     * @throws IllegalArgumentException if the JSON is missing required fields.
     */
    fun fromJson(json: String): WaypointMission {
        val root = gson.fromJson(json, JsonObject::class.java)

        val speed = when {
            root.has("speed_ms")         -> root.get("speed_ms").asFloat
            root.has("default_speed_ms") -> root.get("default_speed_ms").asFloat
            else                         -> 5f
        }.coerceIn(1f, 15f)

        val waypointsJson: JsonArray = root.getAsJsonArray("waypoints")
            ?: throw IllegalArgumentException("JSON has no 'waypoints' array")

        val points = waypointsJson.map { el ->
            val obj = el.asJsonObject
            val lat = obj.get("lat")?.asDouble
                ?: throw IllegalArgumentException("Waypoint missing 'lat'")
            val lon = obj.get("lon")?.asDouble
                ?: throw IllegalArgumentException("Waypoint missing 'lon'")
            val alt = obj.get("alt_m")?.asDouble ?: 30.0
            Triple(lat, lon, alt)
        }

        if (points.size < 2)
            throw IllegalArgumentException("Need at least 2 waypoints (got ${points.size})")

        return DjiSdkManager.buildMission(points, speed)
    }

    /**
     * Convenience: extract just the waypoint list from JSON for display purposes.
     * Returns empty list on parse failure.
     */
    fun waypointCount(json: String): Int = try {
        val root = gson.fromJson(json, JsonObject::class.java)
        root.getAsJsonArray("waypoints")?.size() ?: 0
    } catch (_: Exception) { 0 }
}
