package com.dji.photogrammetry.app

import android.os.Bundle
import android.view.View
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityRouteFollowBinding
import com.google.gson.GsonBuilder

class RouteFollowActivity : AppCompatActivity() {

    private lateinit var binding: ActivityRouteFollowBinding
    private lateinit var profile: DroneProfile
    private val waypoints = mutableListOf<RoutePoint>()

    data class RoutePoint(val name: String, val lat: Double, val lon: Double, val altM: Double)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityRouteFollowBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        val droneId = intent.getStringExtra(MainActivity.EXTRA_DRONE_ID) ?: "mini3"
        profile = DroneProfiles.byId(droneId)

        // Pre-populate with a demo route
        waypoints.addAll(listOf(
            RoutePoint("Start",  47.3769, 8.5417, 30.0),
            RoutePoint("Mid-1",  47.3785, 8.5440, 30.0),
            RoutePoint("Mid-2",  47.3800, 8.5460, 30.0),
            RoutePoint("Finish", 47.3805, 8.5480, 30.0)
        ))
        refreshList()
        setupButtons()
    }

    private fun refreshList() {
        val total = waypoints.zipWithNext().sumOf { (a, b) ->
            MissionEngine.haversine(a.lat, a.lon, b.lat, b.lon)
        }
        binding.tvRouteSummary.text =
            "${waypoints.size} waypoints  |  ~${total.toInt()} m total"
        binding.tvWaypoints.text = waypoints.mapIndexed { i, wp ->
            "${i+1}. ${wp.name}  (${wp.lat}, ${wp.lon})  alt ${wp.altM} m"
        }.joinToString("\n")
    }

    private fun setupButtons() {
        binding.btnAddWaypoint.setOnClickListener {
            showAddWaypointDialog()
        }

        binding.btnClearWaypoints.setOnClickListener {
            waypoints.clear()
            refreshList()
        }

        binding.btnExportRoute.setOnClickListener {
            if (waypoints.size < 2) {
                Toast.makeText(this, "Need at least 2 waypoints", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            val speed       = binding.seekRouteSpeed.progress + 1
            val backwards   = binding.switchBackwards.isChecked
            val updateHome  = binding.switchUpdateHome.isChecked

            val json = buildRouteJson(speed, backwards, updateHome)
            binding.cardExport.visibility = View.VISIBLE
            binding.tvRouteJson.text = json
            Toast.makeText(this, "Route exported — copy or share below", Toast.LENGTH_SHORT).show()
        }

        binding.seekRouteSpeed.setOnSeekBarChangeListener(
            object : android.widget.SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(s: android.widget.SeekBar, p: Int, u: Boolean) {
                    binding.tvSpeedVal.text = "${p + 1} m/s"
                }
                override fun onStartTrackingTouch(s: android.widget.SeekBar) {}
                override fun onStopTrackingTouch(s: android.widget.SeekBar) {}
            }
        )
        binding.seekRouteSpeed.progress = 4
        binding.tvSpeedVal.text = "5 m/s"
    }

    private fun showAddWaypointDialog() {
        val et = android.widget.EditText(this).apply {
            hint = "name, lat, lon, alt  (e.g. WP5, 47.381, 8.549, 30)"
            inputType = android.text.InputType.TYPE_CLASS_TEXT
        }
        androidx.appcompat.app.AlertDialog.Builder(this)
            .setTitle("Add Waypoint")
            .setView(et)
            .setPositiveButton("Add") { _, _ ->
                val parts = et.text.toString().split(",").map { it.trim() }
                if (parts.size >= 4) {
                    val lat = parts[1].toDoubleOrNull()
                    val lon = parts[2].toDoubleOrNull()
                    val alt = parts[3].toDoubleOrNull()
                    if (lat != null && lon != null && alt != null) {
                        waypoints.add(RoutePoint(parts[0], lat, lon, alt))
                        refreshList()
                    } else {
                        Toast.makeText(this, "Invalid coordinates", Toast.LENGTH_SHORT).show()
                    }
                } else {
                    Toast.makeText(this, "Format: name, lat, lon, alt", Toast.LENGTH_SHORT).show()
                }
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    private fun buildRouteJson(speedMs: Int, backwards: Boolean, updateHome: Boolean): String {
        val sb = StringBuilder()
        sb.appendLine("{")
        sb.appendLine("  \"type\": \"follow_route\",")
        sb.appendLine("  \"drone\": \"${profile.id}\",")
        sb.appendLine("  \"default_speed_ms\": $speedMs,")
        sb.appendLine("  \"fly_backwards\": $backwards,")
        sb.appendLine("  \"update_home_at_end\": $updateHome,")
        sb.appendLine("  \"waypoints\": [")
        waypoints.forEachIndexed { i, wp ->
            val comma = if (i < waypoints.size - 1) "," else ""
            sb.appendLine("    {\"name\": \"${wp.name}\", \"lat\": ${wp.lat}, " +
                "\"lon\": ${wp.lon}, \"alt_m\": ${wp.altM}}$comma")
        }
        sb.appendLine("  ]")
        sb.append("}")
        return sb.toString()
    }

    override fun onSupportNavigateUp(): Boolean { finish(); return true }
}
