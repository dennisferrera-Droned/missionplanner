package com.dji.photogrammetry.app

import android.content.Intent
import android.os.Bundle
import android.view.Menu
import android.view.MenuItem
import android.widget.ArrayAdapter
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityMainBinding

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding

    // Persists selected drone across the session
    var selectedProfile: DroneProfile = DroneProfiles.byId("mini3")

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)

        setupDronePicker()
        setupButtons()
    }

    private fun setupDronePicker() {
        val names = DroneProfiles.all.map { it.displayName }
        val adapter = ArrayAdapter(this, android.R.layout.simple_spinner_item, names)
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.spinnerDrone.adapter = adapter

        binding.spinnerDrone.setSelection(0)
        updateProfileCard(DroneProfiles.all[0])

        binding.spinnerDrone.onItemSelectedListener =
            object : android.widget.AdapterView.OnItemSelectedListener {
                override fun onItemSelected(
                    parent: android.widget.AdapterView<*>?, view: android.view.View?,
                    position: Int, id: Long
                ) {
                    selectedProfile = DroneProfiles.all[position]
                    updateProfileCard(selectedProfile)
                }
                override fun onNothingSelected(parent: android.widget.AdapterView<*>?) {}
            }
    }

    private fun updateProfileCard(p: DroneProfile) {
        binding.tvDroneDetails.text = buildString {
            appendLine("📷  ${p.imageWidthPx} × ${p.imageHeightPx} px")
            appendLine("🔭  Focal: ${p.focalLengthMm} mm  |  Sensor: ${p.sensorWidthMm}×${p.sensorHeightMm} mm")
            appendLine("🚀  Max speed: ${p.maxSpeedMs} m/s  |  Max alt: ${p.maxAltitudeM} m")
            append("🛰  RTK: ${if (p.hasRtk) "Built-in ✓" else "None (use GCPs)"}  |  SDK: ${if (p.requiresMsdkV5) "v5" else "v4/v5"}")
        }
    }

    private fun setupButtons() {
        binding.btnMissionPlanner.setOnClickListener {
            val intent = Intent(this, MissionPlannerActivity::class.java)
            intent.putExtra(EXTRA_DRONE_ID, selectedProfile.id)
            startActivity(intent)
        }

        binding.btnRouteFollow.setOnClickListener {
            val intent = Intent(this, RouteFollowActivity::class.java)
            intent.putExtra(EXTRA_DRONE_ID, selectedProfile.id)
            startActivity(intent)
        }

        binding.btnAbout.setOnClickListener { showAbout() }
    }

    private fun showAbout() {
        AlertDialog.Builder(this)
            .setTitle("DJI Photogrammetry SDK")
            .setMessage(
                "Version 1.0.0\n\n" +
                "Open-source mission planning tool for DJI drones.\n\n" +
                "Supported drones:\n" +
                DroneProfiles.all.joinToString("\n") { "  • ${it.displayName}" } +
                "\n\nExports: JSON · KML\n\n" +
                "Note: This app plans and exports missions. " +
                "Upload to your drone using DJI Pilot 2 or the MSDK v5 integration."
            )
            .setPositiveButton("OK", null)
            .show()
    }

    override fun onCreateOptionsMenu(menu: Menu): Boolean {
        menuInflater.inflate(R.menu.main_menu, menu)
        return true
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        return when (item.itemId) {
            R.id.action_about -> { showAbout(); true }
            else -> super.onOptionsItemSelected(item)
        }
    }

    companion object {
        const val EXTRA_DRONE_ID = "drone_id"
    }
}
