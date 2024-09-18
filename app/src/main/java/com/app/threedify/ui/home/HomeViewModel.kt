package com.app.threedify.ui.home

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.chaquo.python.Python

class HomeViewModel : ViewModel() {

    private val _text = MutableLiveData<String>().apply {
        val mypython = Python.getInstance()
        val pythonObj = mypython.getModule("Converter3D")
        val url = "https://jsonplaceholder.typicode.com/posts/1"
        val pythonRes = pythonObj.callAttr("check_trimesh").toString()
        value = pythonRes
        //value = "This is home Fragment"

        val python = Python.getInstance()
        val pythonModule = python.getModule("myTest")
        val result = pythonModule.callAttr("list_installed_packages").toString()
        //value = result
    }
    val text: LiveData<String> = _text
}