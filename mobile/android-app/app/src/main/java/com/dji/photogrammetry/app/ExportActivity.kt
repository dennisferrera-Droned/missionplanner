package com.dji.photogrammetry.app

import android.content.ClipData
import android.content.ClipboardManager
import android.content.Context
import android.content.Intent
import android.os.Bundle
import android.os.Environment
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.dji.photogrammetry.app.databinding.ActivityExportBinding
import java.io.File
import java.text.SimpleDateFormat
import java.util.*

class ExportActivity : AppCompatActivity() {

    private lateinit var binding: ActivityExportBinding
    private var missionJson: String = ""
    private var missionKml:  String = ""
    private var missionSummary: String = ""

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityExportBinding.inflate(layoutInflater)
        setContentView(binding.root)
        setSupportActionBar(binding.toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        missionJson    = intent.getStringExtra(EXTRA_MISSION_JSON)    ?: ""
        missionKml     = intent.getStringExtra(EXTRA_MISSION_KML)     ?: ""
        missionSummary = intent.getStringExtra(EXTRA_MISSION_SUMMARY) ?: ""

        binding.tvSummary.text = missionSummary

        // Show JSON preview
        binding.tvJsonPreview.text =
            if (missionJson.length > 800) missionJson.take(800) + "\n…(truncated)"
            else missionJson

        setupButtons()
    }

    private fun setupButtons() {
        binding.btnCopyJson.setOnClickListener {
            copyToClipboard("Mission JSON", missionJson)
            Toast.makeText(this, "JSON copied to clipboard", Toast.LENGTH_SHORT).show()
        }

        binding.btnSaveJson.setOnClickListener {
            saveFile("mission_${timestamp()}.json", missionJson)
        }

        binding.btnSaveKml.setOnClickListener {
            saveFile("mission_${timestamp()}.kml", missionKml)
        }

        binding.btnShare.setOnClickListener {
            val intent = Intent(Intent.ACTION_SEND).apply {
                type = "text/plain"
                putExtra(Intent.EXTRA_SUBJECT, "DJI Photogrammetry Mission")
                putExtra(Intent.EXTRA_TEXT, missionJson)
            }
            startActivity(Intent.createChooser(intent, "Share Mission"))
        }
    }

    private fun copyToClipboard(label: String, text: String) {
        val clipboard = getSystemService(Context.CLIPBOARD_SERVICE) as ClipboardManager
        clipboard.setPrimaryClip(ClipData.newPlainText(label, text))
    }

    private fun saveFile(name: String, content: String) {
        try {
            val dir = getExternalFilesDir(Environment.DIRECTORY_DOCUMENTS)
                ?: filesDir
            dir.mkdirs()
            val file = File(dir, name)
            file.writeText(content)
            Toast.makeText(this, "Saved: ${file.absolutePath}", Toast.LENGTH_LONG).show()
        } catch (e: Exception) {
            Toast.makeText(this, "Save failed: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    private fun timestamp() =
        SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())

    override fun onSupportNavigateUp(): Boolean { finish(); return true }

    companion object {
        const val EXTRA_MISSION_JSON    = "mission_json"
        const val EXTRA_MISSION_KML     = "mission_kml"
        const val EXTRA_MISSION_SUMMARY = "mission_summary"
    }
}
