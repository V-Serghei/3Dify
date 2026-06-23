package com.example.nativelib

class NativeLib {

    external fun stringFromJNI(): String
    companion object {
        init {
            System.loadLibrary("nativelib")
        }
    }
}