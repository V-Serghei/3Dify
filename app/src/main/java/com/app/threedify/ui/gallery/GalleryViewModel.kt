package com.app.threedify.ui.gallery

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.nativelib.NativeLib

class GalleryViewModel : ViewModel() {
    private val nativeLib = NativeLib()
    private val jniMessage = nativeLib.stringFromJNI()
    private val jniMessage1 = nativeLib.processPointCloud()

    private val _text = MutableLiveData<String>().apply {
        value = "JNI Message: $jniMessage $jniMessage1"
    }
    val text: LiveData<String> = _text
}