package com.app.threedify.ui.gallery

import android.content.ContentValues
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.app.threedify.databinding.FragmentGalleryBinding
import com.example.nativelib.Model3DCreator
import java.io.OutputStream

class GalleryFragment : Fragment() {

    private var _binding: FragmentGalleryBinding? = null
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

        val model3DCreator = Model3DCreator()
        val pointArrays = arrayOf(
            floatArrayOf(0f, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f, 0f),
            floatArrayOf(0f, 0f, 0f, 1f, 0f, 0f, 0.5f, 1f, 1f),
            floatArrayOf(1f, 0f, 0f, 1f, 1f, 0f, 0.5f, 1f, 1f),
            floatArrayOf(1f, 1f, 0f, 0f, 1f, 0f, 0.5f, 1f, 1f),
            floatArrayOf(0f, 0f, 0f, 0f, 1f, 0f, 0.5f, 1f, 1f)
        )


        try {
            textView.text = "1111111111111111111111111111111111111" +
                    "1111111111111111111111111111" +
                    "1111111111111111111" +
                    "11" +
                    "1" +
                    "1" +
                    "1" +
                    "1" +
                    "1" +
                    "1" +
                    "" +
                    "1Point Cloud Processing and Saving: Success"
        } catch (e: Exception) {
            textView.append("\n22222222222222222222222222222222222222Point Cloud Processing: Failed\n${e.message}")
        }

        return root
    }

    private fun saveModelToFile(objData: String, filename: String) {
        val resolver = requireContext().contentResolver
        val values = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, "$filename.obj")
            put(MediaStore.MediaColumns.MIME_TYPE, "application/octet-stream")
            put(MediaStore.MediaColumns.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS)
        }

        val uri = resolver.insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values)
        uri?.let {
            resolver.openOutputStream(it)?.use { outputStream ->
                writeDataToFile(outputStream, objData)
            }
        }
    }

    private fun writeDataToFile(outputStream: OutputStream, data: String) {
        outputStream.write(data.toByteArray())
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
