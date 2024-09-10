package com.app.threedify.ui.gallery

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.app.threedify.databinding.FragmentGalleryBinding
import com.example.nativelib.NativeLib
import com.example.nativelib.Model3DCreator

class GalleryFragment : Fragment() {

    private var _binding: FragmentGalleryBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        val galleryViewModel =
            ViewModelProvider(this).get(GalleryViewModel::class.java)

        _binding = FragmentGalleryBinding.inflate(inflater, container, false)
        val root: View = binding.root
        val textView: TextView = binding.textGallery
        galleryViewModel.text.observe(viewLifecycleOwner) {
            textView.text = it
        }
        val nativeLib = NativeLib()
        val model3DCreator = Model3DCreator()
        val jniMessage = nativeLib.stringFromJNI()
        textView.text = "JNI Message: $jniMessage"

        try {
            model3DCreator.processPointCloud()
            textView.append("\nPoint Cloud Processing: Success")
        } catch (e: Exception) {
            textView.append("\nPoint Cloud Processing: Failed\n${e.message}")
        }
        return root
    }


    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}