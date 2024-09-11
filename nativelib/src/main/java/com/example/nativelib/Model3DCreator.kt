package com.example.nativelib

class Model3DCreator {
    companion object {
        init {
            System.loadLibrary("nativelib")
        }
    }

    external fun processPointCloud(pointArrays: Array<FloatArray>): String
}
