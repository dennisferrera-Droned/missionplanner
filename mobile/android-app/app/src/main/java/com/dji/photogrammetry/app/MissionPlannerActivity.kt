package com.dji.photogrammetry.app

import android.content.Intent
import android.os.Bundle
import android.view.View
import android.widget.AdapterView
import android.widget.ArrayAdapter
import android.widget.SeekBar
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityMissionPlannerBinding

class MissionPlannerActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMissionPlannerBinding
    private lateinit var profile: DroneProfile
    private var lastMission: Mission? = null
    private var roiVertices = mutableListOf<LatLon>()

    // Default ROI — city block near Zurich (~500 m × 400 m)
    private val defaultRoi = listOf(
        LatLon(47.3769, 8.5417),
        LatLon(47.3769, 8.5480),
        LatLon(47.3805, 8.5480),
        LatLon(47.3805, 8.5417)
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMissionPlannerBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        val droneId = intent.getStringExtra(MainActivity.EXTRA_DRONE_ID) ?: "mini3"
        profile = DroneProfiles.byId(droneId)
        roiVertices.addAll(defaultRoi)

        setupPatternSpinner()
        setupSliders()
        setupRoiInput()
        setupButtons()
        updateSliderLabels()
    }

    private fun setupPatternSpinner() {
        val adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_item,
            FlightPattern.values().map { it.label }
        )
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        binding.spinnerPattern.adapter = adapter
    }

    private fun setupSliders() {
        // Altitude: 10–300 m (step 5)
        binding.seekAltitude.max = 58  // (300-10)/5
        binding.seekAltitude.progress = 14  // default 80 m → (80-10)/5=14
        binding.seekAltitude.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        // Speed: 1–15 m/s
        binding.seekSpeed.max = 14
        binding.seekSpeed.progress = 7  // default 8 m/s
        binding.seekSpeed.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        // Front overlap: 50–95 %
        binding.seekFrontOverlap.max = 45
        binding.seekFrontOverlap.progress = 30  // default 80 %
        binding.seekFrontOverlap.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        // Side overlap: 40–90 %
        binding.seekSideOverlap.max = 50
        binding.seekSideOverlap.progress = 30  // default 70 %
        binding.seekSideOverlap.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })
    }

    private fun sliderListener(onChange: () -> Unit) = object : SeekBar.OnSeekBarChangeListener {
        override fun onProgressChanged(s: SeekBar, p: Int, u: Boolean) = onChange()
        override fun onStartTrackingTouch(s: SeekBar) {}
        override fun onStopTrackingTouch(s: SeekBar) {}
    }

    private fun altitude() = 10 + binding.seekAltitude.progress * 5
    private fun speed()    = 1  + binding.seekSpeed.progress
    private fun frontOvlp()= 50 + binding.seekFrontOverlap.progress
    private fun sideOvlp() = 40 + binding.seekSideOverlap.progress

    private fun updateSliderLabels() {
        val alt   = altitude().toDouble()
        val spd   = speed().toDouble()
        val gsdCm = MissionEngine.gsd(profile, alt)
        val (photoDist, photoT) = MissionEngine.photoInterval(profile, alt, frontOvlp().toDouble(), spd)

        binding.tvAltitudeVal.text    = "${altitude()} m"
        binding.tvSpeedVal.text       = "${speed()} m/s"
        binding.tvFrontOverlapVal.text = "${frontOvlp()} %"
        binding.tvSideOverlapVal.text  = "${sideOvlp()} %"
        binding.tvGsd.text            = "GSD: ${"%.2f".format(gsdCm)} cm/px"
        binding.tvPhotoInterval.text  =
            "Photo every ${"%.1f".format(photoDist)} m (${"%.1f".format(photoT)} s at ${speed()} m/s)"
    }

    private fun setupRoiInput() {
        refreshRoiDisplay()

        binding.btnEditRoi.setOnClickListener {
            val dlgBinding = layoutInflater.inflate(R.layout.dialog_roi_input, null)
            val et = dlgBinding.findViewById<android.widget.EditText>(R.id.etRoiCoords)
            et.setText(roiVertices.joinToString("\n") { "${it.lat}, ${it.lon}" })

            androidx.appcompat.app.AlertDialog.Builder(this)
                .setTitle("Enter ROI Corners (lat, lon per line)")
                .setView(dlgBinding)
                .setPositiveButton("Apply") { _, _ ->
                    val parsed = parseRoi(et.text.toString())
                    if (parsed.size >= 3) {
                        roiVertices.clear()
                        roiVertices.addAll(parsed)
                        refreshRoiDisplay()
                        Toast.makeText(this, "ROI updated (${parsed.size} vertices)", Toast.LENGTH_SHORT).show()
                    } else {
                        Toast.makeText(this, "Need at least 3 valid lat/lon pairs", Toast.LENGTH_LONG).show()
                    }
                }
                .setNegativeButton("Cancel", null)
                .show()
        }
    }

    private fun parseRoi(text: String): List<LatLon> {
        return text.lines()
            .mapNotNull { line ->
                val parts = line.split(",").map { it.trim() }
                if (parts.size >= 2) {
                    val lat = parts[0].toDoubleOrNull()
                    val lon = parts[1].toDoubleOrNull()
                    if (lat != null && lon != null) LatLon(lat, lon) else null
                } else null
            }
    }

    private fun refreshRoiDisplay() {
        val roi = RoiPolygon(roiVertices)
        val widthM  = MissionEngine.haversine(roi.minLat, roi.minLon, roi.minLat, roi.maxLon)
        val heightM = MissionEngine.haversine(roi.minLat, roi.minLon, roi.maxLat, roi.minLon)
        binding.tvRoiSummary.text =
            "${roiVertices.size} corners  |  ~${widthM.toInt()} m × ${heightM.toInt()} m  " +
            "(${String.format("%.2f", widthM * heightM / 10_000.0)} ha)"
    }

    private fun setupButtons() {
        binding.btnGenerate.setOnClickListener { generateMission() }
        binding.btnExport.setOnClickListener {
            lastMission?.let { m ->
                val intent = Intent(this, ExportActivity::class.java)
                intent.putExtra(ExportActivity.EXTRA_MISSION_JSON,
                    MissionEngine.missionToJson(m, roiVertices))
                intent.putExtra(ExportActivity.EXTRA_MISSION_KML,
                    MissionEngine.missionToKml(m, roiVertices))
                intent.putExtra(ExportActivity.EXTRA_MISSION_SUMMARY, buildSummary(m))
                startActivity(intent)
            } ?: Toast.makeText(this, "Generate a mission first", Toast.LENGTH_SHORT).show()
        }
    }

    private fun generateMission() {
        if (roiVertices.size < 3) {
            Toast.makeText(this, "Need at least 3 ROI corners", Toast.LENGTH_SHORT).show()
            return
        }
        val alt   = altitude().toDouble()
        val spd   = speed().toDouble()
        val front = frontOvlp().toDouble()
        val side  = sideOvlp().toDouble()
        val pattern = FlightPattern.values()[binding.spinnerPattern.selectedItemPosition]
        val roi   = RoiPolygon(roiVertices)

        val validation = MissionEngine.validate(profile, alt, spd, front, side)

        // Show errors and abort
        if (!validation.isValid) {
            binding.tvValidation.visibility = View.VISIBLE
            binding.tvValidation.text = "⛔  " + validation.errors.joinToString("\n⛔  ")
            binding.tvValidation.setTextColor(getColor(android.R.color.holo_red_dark))
            return
        }

        // Show warnings
        if (validation.warnings.isNotEmpty()) {
            binding.tvValidation.visibility = View.VISIBLE
            binding.tvValidation.text = "⚠️  " + validation.warnings.joinToString("\n⚠️  ")
            binding.tvValidation.setTextColor(getColor(android.R.color.holo_orange_dark))
        } else {
            binding.tvValidation.visibility = View.GONE
        }

        val mission = MissionEngine.plan(roi, pattern, profile, alt, spd, front, side)
        lastMission = mission

        binding.cardResult.visibility = View.VISIBLE
        binding.tvResult.text = buildSummary(mission)
        binding.btnExport.isEnabled = true
    }

    private fun buildSummary(m: Mission): String = buildString {
        appendLine("✅  Mission ready")
        appendLine("Pattern      : ${m.pattern.label}")
        appendLine("Drone        : ${m.droneProfile.displayName}")
        appendLine("Altitude     : ${m.altitudeM} m AGL")
        appendLine("Speed        : ${m.speedMs} m/s")
        appendLine("GSD          : ${"%.2f".format(m.gsdCm)} cm/px")
        appendLine("Coverage     : ${"%.2f".format(m.coverageAreaHa)} ha")
        appendLine("Waypoints    : ${m.waypoints.size}")
        appendLine("Est. photos  : ${m.estimatedPhotos}")
        appendLine("Photo dist   : ${"%.1f".format(m.photoDistanceM)} m")
        append("Photo interval: ${"%.1f".format(m.photoIntervalS)} s")
    }

    override fun onSupportNavigateUp(): Boolean {
        finish()
        return true
    }
}
