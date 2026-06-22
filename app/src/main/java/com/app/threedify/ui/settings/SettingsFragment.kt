package com.app.threedify.ui.settings

import android.content.Context
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.appcompat.app.AppCompatDelegate
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
        setupTheme()
        setupScanQuality()
        return binding.root
    }

    private fun setupTheme() {
        val prefs = requireContext().getSharedPreferences("ThemePrefs", Context.MODE_PRIVATE)
        binding.switchDarkTheme.isChecked = prefs.getBoolean("isDarkTheme", false)
        binding.switchDarkTheme.setOnCheckedChangeListener { _, isChecked ->
            val targetMode = if (isChecked) AppCompatDelegate.MODE_NIGHT_YES
                             else AppCompatDelegate.MODE_NIGHT_NO
            // Guard against spurious calls during view-state restore after recreation
            if (AppCompatDelegate.getDefaultNightMode() == targetMode) return@setOnCheckedChangeListener
            prefs.edit().putBoolean("isDarkTheme", isChecked).apply()
            AppCompatDelegate.setDefaultNightMode(targetMode)
        }
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

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
