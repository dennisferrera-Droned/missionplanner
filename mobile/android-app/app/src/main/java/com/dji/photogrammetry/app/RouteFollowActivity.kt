package com.dji.photogrammetry.app

import android.content.Intent
import android.net.Uri
import android.os.Bundle
import android.util.Xml
import android.view.View
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityRouteFollowBinding
import org.xmlpull.v1.XmlPullParser
import java.io.InputStream
import java.util.zip.ZipInputStream

class RouteFollowActivity : AppCompatActivity() {

    private lateinit var binding: ActivityRouteFollowBinding
    private lateinit var profile: DroneProfile
    private val waypoints = mutableListOf<RoutePoint>()

    data class RoutePoint(val name: String, val lat: Double, val lon: Double, val altM: Double)

    // ---------------------------------------------------------------------------
    // File picker — launched when the user taps "Import KML / KMZ"
    // ---------------------------------------------------------------------------
    private val kmlPickerLauncher = registerForActivityResult(
        ActivityResultContracts.StartActivityForResult()
    ) { result ->
        val uri = result.data?.data ?: return@registerForActivityResult
        importKmlOrKmz(uri)
    }

    // ---------------------------------------------------------------------------
    // Lifecycle
    // ---------------------------------------------------------------------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityRouteFollowBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        val droneId = intent.getStringExtra(MainActivity.EXTRA_DRONE_ID) ?: "mini3"
        profile = DroneProfiles.byId(droneId)

        waypoints.addAll(listOf(
            RoutePoint("Start",  47.3769, 8.5417, 30.0),
            RoutePoint("Mid-1",  47.3785, 8.5440, 30.0),
            RoutePoint("Mid-2",  47.3800, 8.5460, 30.0),
            RoutePoint("Finish", 47.3805, 8.5480, 30.0)
        ))

        setupSliders()
        setupButtons()
        refreshList()
    }

    // ---------------------------------------------------------------------------
    // Sliders
    // ---------------------------------------------------------------------------

    private fun altitude() = 10 + binding.seekAltitude.progress * 5   // 10–300 m, step 5

    private fun setupSliders() {
        // Speed 1–15 m/s
        binding.seekRouteSpeed.max = 14
        binding.seekRouteSpeed.progress = 4
        binding.tvSpeedVal.text = "5 m/s"
        binding.seekRouteSpeed.setOnSeekBarChangeListener(
            object : android.widget.SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(s: android.widget.SeekBar, p: Int, u: Boolean) {
                    binding.tvSpeedVal.text = "${p + 1} m/s"
                }
                override fun onStartTrackingTouch(s: android.widget.SeekBar) {}
                override fun onStopTrackingTouch(s: android.widget.SeekBar) {}
            }
        )

        // Altitude 10–300 m (step 5) — default 30 m → progress = (30-10)/5 = 4
        binding.seekAltitude.max = 58
        binding.seekAltitude.progress = 4
        binding.tvAltitudeVal.text = "30 m"
        binding.seekAltitude.setOnSeekBarChangeListener(
            object : android.widget.SeekBar.OnSeekBarChangeListener {
                override fun onProgressChanged(s: android.widget.SeekBar, p: Int, u: Boolean) {
                    binding.tvAltitudeVal.text = "${altitude()} m"
                }
                override fun onStartTrackingTouch(s: android.widget.SeekBar) {}
                override fun onStopTrackingTouch(s: android.widget.SeekBar) {}
            }
        )
    }

    // ---------------------------------------------------------------------------
    // Buttons
    // ---------------------------------------------------------------------------

    private fun setupButtons() {
        binding.btnAddWaypoint.setOnClickListener { showAddWaypointDialog() }

        binding.btnClearWaypoints.setOnClickListener {
            waypoints.clear()
            refreshList()
        }

        // Apply the current altitude slider value to every existing waypoint
        binding.btnApplyAltitude.setOnClickListener {
            val alt = altitude().toDouble()
            val updated = waypoints.map { it.copy(altM = alt) }
            waypoints.clear()
            waypoints.addAll(updated)
            refreshList()
            Toast.makeText(this, "Altitude set to ${alt.toInt()} m for all waypoints", Toast.LENGTH_SHORT).show()
        }

        // KML / KMZ import
        binding.btnImportKml.setOnClickListener {
            val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                addCategory(Intent.CATEGORY_OPENABLE)
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

        binding.btnExportRoute.setOnClickListener {
            if (waypoints.size < 2) {
                Toast.makeText(this, "Need at least 2 waypoints", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }
            val speed      = binding.seekRouteSpeed.progress + 1
            val backwards  = binding.switchBackwards.isChecked
            val updateHome = binding.switchUpdateHome.isChecked
            val json       = buildRouteJson(speed, backwards, updateHome)
            binding.cardExport.visibility = View.VISIBLE
            binding.tvRouteJson.text = json
            Toast.makeText(this, "Route exported — copy or share below", Toast.LENGTH_SHORT).show()
        }
    }

    // ---------------------------------------------------------------------------
    // Add waypoint dialog
    // ---------------------------------------------------------------------------

    private fun showAddWaypointDialog() {
        val defaultAlt = altitude().toDouble()
        val et = android.widget.EditText(this).apply {
            hint = "name, lat, lon  (e.g. WP5, 47.381, 8.549)"
            inputType = android.text.InputType.TYPE_CLASS_TEXT
        }
        AlertDialog.Builder(this)
            .setTitle("Add Waypoint  (alt: ${defaultAlt.toInt()} m from slider)")
            .setView(et)
            .setPositiveButton("Add") { _, _ ->
                val parts = et.text.toString().split(",").map { it.trim() }
                if (parts.size >= 3) {
                    val lat = parts[1].toDoubleOrNull()
                    val lon = parts[2].toDoubleOrNull()
                    // Optional 4th field overrides altitude; otherwise use slider
                    val alt = if (parts.size >= 4) parts[3].toDoubleOrNull() ?: defaultAlt
                              else defaultAlt
                    if (lat != null && lon != null) {
                        waypoints.add(RoutePoint(parts[0], lat, lon, alt))
                        refreshList()
                    } else {
                        Toast.makeText(this, "Invalid coordinates", Toast.LENGTH_SHORT).show()
                    }
                } else {
                    Toast.makeText(this, "Format: name, lat, lon  (optional: alt)", Toast.LENGTH_SHORT).show()
                }
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    // ---------------------------------------------------------------------------
    // KML / KMZ import
    // ---------------------------------------------------------------------------

    /**
     * Entry point for a URI returned by the file picker.
     * Detects KMZ (ZIP) vs plain KML, then delegates to the XML parser.
     */
    private fun importKmlOrKmz(uri: Uri) {
        try {
            val fileName = uri.lastPathSegment?.lowercase() ?: ""
            val stream   = contentResolver.openInputStream(uri)
                ?: throw IllegalStateException("Cannot open file")

            val imported = if (fileName.endsWith(".kmz") || isZip(uri)) {
                parseKmzWaypoints(stream)
            } else {
                parseKmlWaypoints(stream)
            }

            when {
                imported.isEmpty() ->
                    showError(
                        "No waypoints found",
                        "The file contains no recognisable points.\n\n" +
                        "Supported KML elements:\n" +
                        "  • <Placemark><Point> — individual waypoints\n" +
                        "  • <LineString><coordinates> — path as waypoints\n" +
                        "  • <Polygon> outer boundary — polygon vertices as waypoints"
                    )
                else -> {
                    val alt = altitude().toDouble()
                    val withAlt = imported.map { it.copy(altM = alt) }
                    showImportModeDialog(withAlt)
                }
            }
        } catch (e: Exception) {
            showError("Import failed", e.message ?: "Unknown error")
        }
    }

    /**
     * Shows a dialog asking whether to replace or append to existing waypoints,
     * then applies the choice.
     */
    private fun showImportModeDialog(imported: List<RoutePoint>) {
        val altLabel = altitude()
        AlertDialog.Builder(this)
            .setTitle("Import ${imported.size} waypoints  (alt: ${altLabel} m)")
            .setMessage(
                "What would you like to do with the existing ${waypoints.size} waypoint(s)?"
            )
            .setPositiveButton("Replace all") { _, _ ->
                waypoints.clear()
                waypoints.addAll(imported)
                refreshList()
                Toast.makeText(
                    this, "Imported ${imported.size} waypoints", Toast.LENGTH_SHORT
                ).show()
            }
            .setNeutralButton("Append") { _, _ ->
                waypoints.addAll(imported)
                refreshList()
                Toast.makeText(
                    this, "Appended ${imported.size} waypoints (total: ${waypoints.size})",
                    Toast.LENGTH_SHORT
                ).show()
            }
            .setNegativeButton("Cancel", null)
            .show()
    }

    /** Sniff first 4 bytes for ZIP magic (PK\x03\x04). */
    private fun isZip(uri: Uri): Boolean {
        return try {
            contentResolver.openInputStream(uri)?.use { s ->
                val m = ByteArray(4); s.read(m)
                m[0] == 0x50.toByte() && m[1] == 0x4B.toByte()
            } ?: false
        } catch (_: Exception) { false }
    }

    /** Unzip KMZ, find first .kml entry, parse it. */
    private fun parseKmzWaypoints(stream: InputStream): List<RoutePoint> {
        ZipInputStream(stream).use { zip ->
            var entry = zip.nextEntry
            while (entry != null) {
                if (!entry.isDirectory && entry.name.lowercase().endsWith(".kml")) {
                    return parseKmlWaypoints(zip.readBytes().inputStream())
                }
                entry = zip.nextEntry
            }
        }
        return emptyList()
    }

    /**
     * Parse KML and extract waypoints from:
     *   1. <Placemark><Point><coordinates>        → one waypoint per Placemark
     *   2. <LineString><coordinates>              → each vertex becomes a waypoint
     *   3. <Polygon><outerBoundaryIs><LinearRing> → polygon corners as waypoints
     *
     * KML coordinate order is lon,lat[,alt].
     */
    private fun parseKmlWaypoints(stream: InputStream): List<RoutePoint> {
        val parser = Xml.newPullParser()
        parser.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false)
        parser.setInput(stream, null)

        val results = mutableListOf<RoutePoint>()

        var currentName      = ""
        var inPlacemark      = false
        var inPoint          = false
        var inLineString     = false
        var inOuterBoundary  = false
        var inLinearRing     = false
        var placemarkIndex   = 0

        var event = parser.eventType
        while (event != XmlPullParser.END_DOCUMENT) {
            when (event) {
                XmlPullParser.START_TAG -> when (parser.name) {
                    "Placemark" -> {
                        inPlacemark = true
                        currentName = ""
                    }
                    "name" -> if (inPlacemark) {
                        currentName = parser.nextText().trim()
                    }
                    "Point"          -> inPoint         = true
                    "LineString"     -> inLineString    = true
                    "outerBoundaryIs"-> inOuterBoundary = true
                    "LinearRing"     -> inLinearRing    = true
                    "coordinates"    -> {
                        val raw = parser.nextText().trim()
                        when {
                            // Single-point Placemark
                            inPoint -> {
                                singleCoord(raw)?.let { (lat, lon) ->
                                    val label = currentName.ifBlank { "WP${++placemarkIndex}" }
                                    results.add(RoutePoint(label, lat, lon, 0.0))
                                }
                            }
                            // LineString — each vertex is a waypoint
                            inLineString -> {
                                val pts = coordList(raw)
                                pts.forEachIndexed { i, (lat, lon) ->
                                    val label = if (currentName.isBlank()) "LS${i+1}"
                                                else "${currentName}_${i+1}"
                                    results.add(RoutePoint(label, lat, lon, 0.0))
                                }
                            }
                            // Polygon outer boundary
                            inOuterBoundary && inLinearRing -> {
                                val pts = coordList(raw)
                                pts.forEachIndexed { i, (lat, lon) ->
                                    val label = if (currentName.isBlank()) "V${i+1}"
                                                else "${currentName}_${i+1}"
                                    results.add(RoutePoint(label, lat, lon, 0.0))
                                }
                            }
                        }
                    }
                }
                XmlPullParser.END_TAG -> when (parser.name) {
                    "Placemark"       -> { inPlacemark = false; inPoint = false }
                    "Point"           -> inPoint          = false
                    "LineString"      -> inLineString     = false
                    "outerBoundaryIs" -> inOuterBoundary  = false
                    "LinearRing"      -> inLinearRing     = false
                }
            }
            event = parser.next()
        }
        return results
    }

    /**
     * Parse a single "lon,lat[,alt]" KML coordinate token.
     * Returns Pair(lat, lon) — note the swap.
     */
    private fun singleCoord(raw: String): Pair<Double, Double>? {
        val token = raw.trim().split(Regex("\\s+")).firstOrNull() ?: return null
        val parts = token.split(",")
        if (parts.size < 2) return null
        val lon = parts[0].toDoubleOrNull() ?: return null
        val lat = parts[1].toDoubleOrNull() ?: return null
        return Pair(lat, lon)
    }

    /**
     * Parse a whitespace-separated list of "lon,lat[,alt]" KML coordinates.
     * Returns list of Pair(lat, lon) with closing duplicate removed.
     */
    private fun coordList(raw: String): List<Pair<Double, Double>> {
        val pts = raw.split(Regex("\\s+")).mapNotNull { token ->
            val parts = token.split(",")
            if (parts.size >= 2) {
                val lon = parts[0].toDoubleOrNull()
                val lat = parts[1].toDoubleOrNull()
                if (lon != null && lat != null) Pair(lat, lon) else null
            } else null
        }
        // Drop KML closing duplicate
        return if (pts.size >= 2 && pts.first() == pts.last()) pts.dropLast(1) else pts
    }

    // ---------------------------------------------------------------------------
    // Display
    // ---------------------------------------------------------------------------

    private fun refreshList() {
        val total = waypoints.zipWithNext().sumOf { (a, b) ->
            MissionEngine.haversine(a.lat, a.lon, b.lat, b.lon)
        }
        binding.tvRouteSummary.text =
            "${waypoints.size} waypoints  |  ~${total.toInt()} m total"
        binding.tvWaypoints.text = waypoints.mapIndexed { i, wp ->
            "${i+1}. ${wp.name}  (${wp.lat}, ${wp.lon})  ${wp.altM.toInt()} m"
        }.joinToString("\n")
    }

    // ---------------------------------------------------------------------------
    // Export
    // ---------------------------------------------------------------------------

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
            sb.appendLine(
                "    {\"name\": \"${wp.name}\", \"lat\": ${wp.lat}, " +
                "\"lon\": ${wp.lon}, \"alt_m\": ${wp.altM}}$comma"
            )
        }
        sb.appendLine("  ]")
        sb.append("}")
        return sb.toString()
    }

    // ---------------------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------------------

    private fun showError(title: String, message: String) {
        AlertDialog.Builder(this)
            .setTitle(title)
            .setMessage(message)
            .setPositiveButton("OK", null)
            .show()
    }

    override fun onSupportNavigateUp(): Boolean { finish(); return true }
}
