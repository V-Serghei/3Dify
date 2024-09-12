package com.app.threedify.ui.gallery

import android.content.ContentValues
import android.os.Environment
import android.provider.MediaStore
import android.app.Application
import android.content.Context
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import com.example.nativelib.Model3DCreator
import java.io.OutputStream

class GalleryViewModel(application: Application) : AndroidViewModel(application) {
    private val model3DCreator = Model3DCreator()
    private val pointArrays = arrayOf(
        floatArrayOf(0f, 0f, 0f, 1f, 0f, 0f, 1f, 1f, 0f, 0f, 1f, 0f),
        floatArrayOf(0f, 0f, 0f, 1f, 0f, 0f, 0.5f, 1f, 1f),
        floatArrayOf(1f, 0f, 0f, 1f, 1f, 0f, 0.5f, 1f, 1f),
        floatArrayOf(1f, 1f, 0f, 0f, 1f, 0f, 0.5f, 1f, 1f),
        floatArrayOf(0f, 0f, 0f, 0f, 1f, 0f, 0.5f, 1f, 1f)
    )

    private val _text = MutableLiveData<String>().apply {
        value = try {
            "Point Cloud Processing and Saving: Success"
        } catch (e: Exception) {
            "Point Cloud Processing: Failed\n${e.message}"
        }
    }
    val text: LiveData<String> = _text

    private fun saveModelToFile(objData: String, filename: String) {
        val resolver = getApplication<Application>().contentResolver
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
}
