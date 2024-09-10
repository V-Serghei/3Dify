package com.app.threedify.ui.gallery

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.nativelib.NativeLib
import com.example.nativelib.Model3DCreator

class GalleryViewModel : ViewModel() {
    private val nativeLib = NativeLib()
    private val model3DCreator = Model3DCreator()
    private val jniMessage = nativeLib.stringFromJNI()
    private val jniMessage1 = model3DCreator.processPointCloud()

    private val _text = MutableLiveData<String>().apply {
        value = "JNI Message: $jniMessage $jniMessage1"
    }
    val text: LiveData<String> = _text
}