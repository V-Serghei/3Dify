package com.example.nativelib

class Model3DCreator {
    companion object {
        init {
            System.loadLibrary("nativelib")
        }
    }

    external fun processPointCloudToUri(pointArrays: Array<FloatArray>, fileDescriptor: Int): Boolean
}
