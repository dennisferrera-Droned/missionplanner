package com.dji.photogrammetry.app

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.util.Xml
import android.view.View
import android.widget.ArrayAdapter
import android.widget.SeekBar
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityMissionPlannerBinding
import org.xmlpull.v1.XmlPullParser
import java.io.InputStream
import java.util.zip.ZipInputStream

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

    // ---------------------------------------------------------------------------
    // File picker — launched when the user taps "Import KML / KMZ"
    // ---------------------------------------------------------------------------
    private val kmlPickerLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        val uri = result.data?.data ?: return@registerForActivityResult
        importKmlOrKmz(uri)
    }

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

    // ---------------------------------------------------------------------------
    // Setup
    // ---------------------------------------------------------------------------

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
        binding.seekAltitude.max = 58       // 10–300 m, step 5
        binding.seekAltitude.progress = 14  // default 80 m
        binding.seekAltitude.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        binding.seekSpeed.max = 14
        binding.seekSpeed.progress = 7      // default 8 m/s
        binding.seekSpeed.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        binding.seekFrontOverlap.max = 45
        binding.seekFrontOverlap.progress = 30  // default 80 %
        binding.seekFrontOverlap.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })

        binding.seekSideOverlap.max = 50
        binding.seekSideOverlap.progress = 30   // default 70 %
        binding.seekSideOverlap.setOnSeekBarChangeListener(sliderListener { updateSliderLabels() })
    }

    private fun sliderListener(onChange: () -> Unit) = object : SeekBar.OnSeekBarChangeListener {
        override fun onProgressChanged(s: SeekBar, p: Int, u: Boolean) = onChange()
        override fun onStartTrackingTouch(s: SeekBar) {}
        override fun onStopTrackingTouch(s: SeekBar) {}
    }

    private fun altitude()   = 10 + binding.seekAltitude.progress * 5
    private fun speed()      = 1  + binding.seekSpeed.progress
    private fun frontOvlp()  = 50 + binding.seekFrontOverlap.progress
    private fun sideOvlp()   = 40 + binding.seekSideOverlap.progress

    private fun updateSliderLabels() {
        val alt = altitude().toDouble()
        val spd = speed().toDouble()
        val gsdCm = MissionEngine.gsd(profile, alt)
        val (photoDist, photoT) = MissionEngine.photoInterval(
            profile, alt, frontOvlp().toDouble(), spd
        )
        binding.tvAltitudeVal.text     = "${altitude()} m"
        binding.tvSpeedVal.text        = "${speed()} m/s"
        binding.tvFrontOverlapVal.text = "${frontOvlp()} %"
        binding.tvSideOverlapVal.text  = "${sideOvlp()} %"
        binding.tvGsd.text             = "GSD: ${"%.2f".format(gsdCm)} cm/px"
        binding.tvPhotoInterval.text   =
            "Photo every ${"%.1f".format(photoDist)} m " +
            "(${"%.1f".format(photoT)} s at ${speed()} m/s)"
    }

    // ---------------------------------------------------------------------------
    // ROI input
    // ---------------------------------------------------------------------------

    private fun setupRoiInput() {
        refreshRoiDisplay()

        // Manual corner editor
        binding.btnEditRoi.setOnClickListener {
            val dlgBinding = layoutInflater.inflate(R.layout.dialog_roi_input, null)
            val et = dlgBinding.findViewById<android.widget.EditText>(R.id.etRoiCoords)
            et.setText(roiVertices.joinToString("\n") { "${it.lat}, ${it.lon}" })

            AlertDialog.Builder(this)
                .setTitle("Enter ROI Corners (lat, lon per line)")
                .setView(dlgBinding)
                .setPositiveButton("Apply") { _, _ ->
                    val parsed = parseManualRoi(et.text.toString())
                    if (parsed.size >= 3) {
                        applyRoi(parsed, "ROI updated (${parsed.size} corners)")
                    } else {
                        Toast.makeText(
                            this, "Need at least 3 valid lat/lon pairs",
                            Toast.LENGTH_LONG
                        ).show()
                    }
                }
                .setNegativeButton("Cancel", null)
                .show()
        }

        // KML / KMZ file importer
        binding.btnImportKml.setOnClickListener {
            val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                addCategory(Intent.CATEGORY_OPENABLE)
                // Accept KML, KMZ, and generic binary (for files served without a proper MIME type)
                type = "*/*"
                putExtra(
                    Intent.EXTRA_MIME_TYPES, arrayOf(
                        "application/vnd.google-earth.kml+xml",
                        "application/vnd.google-earth.kmz",
                        "application/xml",
                        "text/xml",
                        "application/zip",
                        "application/octet-stream"
                    )
                )
            }
            kmlPickerLauncher.launch(intent)
        }
    }

    // ---------------------------------------------------------------------------
    // KML / KMZ import
    // ---------------------------------------------------------------------------

    /**
     * Entry point for a URI returned by the file picker.
     * Detects whether the file is KMZ (ZIP containing doc.kml) or plain KML,
     * then delegates to the XML parser.
     */
    private fun importKmlOrKmz(uri: Uri) {
        try {
            val fileName = uri.lastPathSegment?.lowercase() ?: ""
            val stream   = contentResolver.openInputStream(uri)
                ?: throw IllegalStateException("Cannot open file")

            val vertices = if (fileName.endsWith(".kmz") || isZip(uri)) {
                parseKmz(stream)
            } else {
                parseKml(stream)
            }

            when {
                vertices.isEmpty() ->
                    showImportError(
                        "No polygon found",
                        "The file does not contain a recognisable polygon boundary.\n\n" +
                        "Make sure the KML/KMZ has a <Polygon> or <LinearRing> element " +
                        "with <coordinates>."
                    )

                vertices.size < 3 ->
                    showImportError(
                        "Polygon too small",
                        "Found only ${vertices.size} point(s). Need at least 3 to define an area."
                    )

                else -> {
                    // If the file has multiple polygons, let the user pick one
                    // (rare — most survey KMLs have one boundary)
                    applyRoi(vertices, "Imported ${vertices.size} corners from KML")
                }
            }
        } catch (e: Exception) {
            showImportError("Import failed", e.message ?: "Unknown error")
        }
    }

    /**
     * Sniff the first 4 bytes to detect a ZIP signature (PK\x03\x04).
     * Used when the file extension is missing or wrong.
     */
    private fun isZip(uri: Uri): Boolean {
        return try {
            contentResolver.openInputStream(uri)?.use { stream ->
                val magic = ByteArray(4)
                stream.read(magic)
                magic[0] == 0x50.toByte() && magic[1] == 0x4B.toByte()
            } ?: false
        } catch (_: Exception) { false }
    }

    /**
     * Parse a KMZ file (ZIP archive).
     * Looks for the first .kml entry — typically "doc.kml" — and parses it.
     */
    private fun parseKmz(stream: InputStream): List<LatLon> {
        ZipInputStream(stream).use { zip ->
            var entry = zip.nextEntry
            while (entry != null) {
                if (!entry.isDirectory && entry.name.lowercase().endsWith(".kml")) {
                    // Read the KML bytes while keeping the ZipInputStream open
                    val bytes = zip.readBytes()
                    return parseKml(bytes.inputStream())
                }
                entry = zip.nextEntry
            }
        }
        return emptyList()
    }

    /**
     * Parse KML XML and extract the **outer boundary** of the first polygon found.
     *
     * KML coordinate format: lon,lat[,alt] separated by whitespace.
     * Note: longitude comes FIRST in KML, opposite to what you might expect.
     *
     * Handles both:
     *   • <Polygon><outerBoundaryIs><LinearRing><coordinates>
     *   • Bare <LinearRing><coordinates> (e.g. ground overlays, paths used as boundaries)
     */
    private fun parseKml(stream: InputStream): List<LatLon> {
        val parser = Xml.newPullParser()
        parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false)
        parser.setInput(stream, null)

        var inOuterBoundary = false
        var insidePolygon   = false
        var insideLinearRing = false
        val results = mutableListOf<List<LatLon>>()

        var event = parser.eventType
        while (event != XmlPullParser.END_DOCUMENT) {
            when (event) {
                XmlPullParser.START_TAG -> when (parser.name) {
                    "Polygon"         -> insidePolygon    = true
                    "outerBoundaryIs" -> inOuterBoundary  = true
                    "LinearRing"      -> insideLinearRing = true
                    "coordinates"     -> {
                        // Only capture coordinates that are inside an outer boundary
                        // (or a bare LinearRing if there is no Polygon wrapper)
                        if (inOuterBoundary || (!insidePolygon && insideLinearRing)) {
                            val raw = parser.nextText().trim()
                            val pts = parseKmlCoordinateString(raw)
                            if (pts.size >= 3) results.add(pts)
                        }
                    }
                }
                XmlPullParser.END_TAG -> when (parser.name) {
                    "Polygon"         -> { insidePolygon = false; inOuterBoundary = false }
                    "outerBoundaryIs" -> inOuterBoundary  = false
                    "LinearRing"      -> insideLinearRing = false
                }
            }
            event = parser.next()
        }

        return results.firstOrNull() ?: emptyList()
    }

    /**
     * Convert a KML coordinate string to a LatLon list.
     *
     * Each tuple is "lon,lat" or "lon,lat,alt" separated by whitespace.
     * KML closes polygons by repeating the first point — we drop the duplicate.
     */
    private fun parseKmlCoordinateString(raw: String): List<LatLon> {
        val points = raw.split(Regex("\\s+"))
            .mapNotNull { token ->
                val parts = token.split(",")
                if (parts.size >= 2) {
                    val lon = parts[0].toDoubleOrNull()
                    val lat = parts[1].toDoubleOrNull()
                    if (lon != null && lat != null) LatLon(lat, lon) else null
                } else null
            }

        // Drop the closing duplicate point if KML closed the ring
        return if (points.size >= 2 &&
            points.first().lat == points.last().lat &&
            points.first().lon == points.last().lon
        ) {
            points.dropLast(1)
        } else {
            points
        }
    }

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    private fun parseManualRoi(text: String): List<LatLon> {
        return text.lines().mapNotNull { line ->
            val parts = line.split(",").map { it.trim() }
            if (parts.size >= 2) {
                val lat = parts[0].toDoubleOrNull()
                val lon = parts[1].toDoubleOrNull()
                if (lat != null && lon != null) LatLon(lat, lon) else null
            } else null
        }
    }

    private fun applyRoi(vertices: List<LatLon>, message: String) {
        roiVertices.clear()
        roiVertices.addAll(vertices)
        refreshRoiDisplay()
        // Reset any stale mission result
        lastMission = null
        binding.cardResult.visibility = View.GONE
        binding.btnExport.isEnabled   = false
        binding.tvValidation.visibility = View.GONE
        Toast.makeText(this, message, Toast.LENGTH_SHORT).show()
    }

    private fun refreshRoiDisplay() {
        if (roiVertices.isEmpty()) {
            binding.tvRoiSummary.text = "No ROI defined"
            return
        }
        val roi    = RoiPolygon(roiVertices)
        val widthM = MissionEngine.haversine(roi.minLat, roi.minLon, roi.minLat, roi.maxLon)
        val heightM = MissionEngine.haversine(roi.minLat, roi.minLon, roi.maxLat, roi.minLon)
        binding.tvRoiSummary.text =
            "${roiVertices.size} corners  |  ~${widthM.toInt()} m × ${heightM.toInt()} m  " +
            "(${String.format("%.2f", widthM * heightM / 10_000.0)} ha)"
    }

    private fun showImportError(title: String, message: String) {
        AlertDialog.Builder(this)
            .setTitle(title)
            .setMessage(message)
            .setPositiveButton("OK", null)
            .show()
    }

    // ---------------------------------------------------------------------------
    // Mission generation & export
    // ---------------------------------------------------------------------------

    private fun setupButtons() {
        binding.btnGenerate.setOnClickListener { generateMission() }
        binding.btnExport.setOnClickListener {
            lastMission?.let { m ->
                val intent = Intent(this, ExportActivity::class.java)
                intent.putExtra(
                    ExportActivity.EXTRA_MISSION_JSON,
                    MissionEngine.missionToJson(m, roiVertices)
                )
                intent.putExtra(
                    ExportActivity.EXTRA_MISSION_KML,
                    MissionEngine.missionToKml(m, roiVertices)
                )
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
        val alt     = altitude().toDouble()
        val spd     = speed().toDouble()
        val front   = frontOvlp().toDouble()
        val side    = sideOvlp().toDouble()
        val pattern = FlightPattern.values()[binding.spinnerPattern.selectedItemPosition]
        val roi     = RoiPolygon(roiVertices)

        val validation = MissionEngine.validate(profile, alt, spd, front, side)

        if (!validation.isValid) {
            binding.tvValidation.visibility = View.VISIBLE
            binding.tvValidation.text =
                "⛔  " + validation.errors.joinToString("\n⛔  ")
            binding.tvValidation.setTextColor(getColor(android.R.color.holo_red_dark))
            return
        }

        if (validation.warnings.isNotEmpty()) {
            binding.tvValidation.visibility = View.VISIBLE
            binding.tvValidation.text =
                "⚠️  " + validation.warnings.joinToString("\n⚠️  ")
            binding.tvValidation.setTextColor(getColor(android.R.color.holo_orange_dark))
        } else {
            binding.tvValidation.visibility = View.GONE
        }

        val mission = MissionEngine.plan(roi, pattern, profile, alt, spd, front, side)
        lastMission = mission

        binding.cardResult.visibility = View.VISIBLE
        binding.tvResult.text         = buildSummary(mission)
        binding.btnExport.isEnabled   = true
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

    override fun onSupportNavigateUp(): Boolean { finish(); return true }
}
