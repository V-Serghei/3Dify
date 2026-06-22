package com.app.threedify.ui.settings

import android.app.AlertDialog
import android.content.Context
import android.os.Bundle
import android.os.Environment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import com.app.threedify.databinding.FragmentSettingsBinding

class SettingsFragment : Fragment() {

    private var _binding: FragmentSettingsBinding? = null
    private val binding get() = _binding!!

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentSettingsBinding.inflate(inflater, container, false)
        setupScanQuality()
        setupSavePath()
        return binding.root
    }

    private fun setupScanQuality() {
        val prefs = requireContext().getSharedPreferences("ScanSettings", Context.MODE_PRIVATE)
        when (prefs.getString("quality", "medium")) {
            "low" -> binding.radioScanQuality.check(binding.qualityLow.id)
            "high" -> binding.radioScanQuality.check(binding.qualityHigh.id)
            else -> binding.radioScanQuality.check(binding.qualityMedium.id)
        }
        binding.radioScanQuality.setOnCheckedChangeListener { _, checkedId ->
            val quality = when (checkedId) {
                binding.qualityLow.id -> "low"
                binding.qualityHigh.id -> "high"
                else -> "medium"
            }
            prefs.edit().putString("quality", quality).apply()
        }
    }

    private fun setupSavePath() {
        val prefs = requireContext().getSharedPreferences("SaveSettings", Context.MODE_PRIVATE)

        fun updateDisplay() {
            val relativePath = prefs.getString("modelRelativePath", Environment.DIRECTORY_DOWNLOADS)
                ?: Environment.DIRECTORY_DOWNLOADS
            binding.textSavePath.text = relativePath.replace("/", " / ")
        }

        updateDisplay()

        binding.rowSavePath.setOnClickListener {
            val options = arrayOf(
                "Downloads",
                "Downloads / 3Dify",
                "Documents / 3Dify"
            )
            val relativePaths = arrayOf(
                Environment.DIRECTORY_DOWNLOADS,
                "${Environment.DIRECTORY_DOWNLOADS}/3Dify",
                "${Environment.DIRECTORY_DOCUMENTS}/3Dify"
            )
            val current = prefs.getString("modelRelativePath", Environment.DIRECTORY_DOWNLOADS)
            val checkedItem = relativePaths.indexOfFirst { it == current }.coerceAtLeast(0)

            AlertDialog.Builder(requireContext())
                .setTitle("Save 3D models to")
                .setSingleChoiceItems(options, checkedItem) { dialog, which ->
                    prefs.edit().putString("modelRelativePath", relativePaths[which]).apply()
                    updateDisplay()
                    dialog.dismiss()
                }
                .setNegativeButton("Cancel", null)
                .show()
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
