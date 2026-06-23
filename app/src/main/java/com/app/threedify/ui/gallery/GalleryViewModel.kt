package com.app.threedify.ui.gallery

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.app.threedify.ui.gallery.helpers.ObjFile

class GalleryViewModel : ViewModel() {

    private val _objFiles = MutableLiveData<List<ObjFile>>()
    val objFiles: LiveData<List<ObjFile>> = _objFiles

    fun updateObjFiles(files: List<ObjFile>) {
        _objFiles.value = files
    }
}