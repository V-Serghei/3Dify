package com.app.threedify.ui.camera

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.fragment.findNavController
import com.app.threedify.databinding.FragmentCameraBinding
import com.app.arcore.ArCoreManager
import com.app.arcore.helpers.CameraPermissionHelper
import com.app.threedify.R

class CameraFragment : Fragment() {

    private var _binding: FragmentCameraBinding? = null
    private val binding get() = _binding!!
    private val arCoreManager = ArCoreManager()

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        val cameraViewModel = ViewModelProvider(this)[CameraViewModel::class.java]

        _binding = FragmentCameraBinding.inflate(inflater, container, false)
        val root: View = binding.root

        arCoreManager.initialize(requireContext(), binding.arSceneView, requireActivity())

        binding.togglePointCloudButton.setOnClickListener {
            arCoreManager.togglePointCloud()
        }

        return root
    }

    override fun onResume() {
        super.onResume()
        arCoreManager.onResume(requireContext(), requireActivity())
    }

    override fun onPause() {
        super.onPause()
        arCoreManager.onPause()
    }

    override fun onDestroyView() {
        super.onDestroyView()
        arCoreManager.onDestroy()
        _binding = null
    }
}
