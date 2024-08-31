package com.example.pclibrarycpp

class NativeLib {

    /**
     * A native method that is implemented by the 'nativelib' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    external fun processPointCloud(pointCloudArray: FloatArray): String

    companion object {
        // Used to load the 'nativelib' library on application startup.
        init {
            System.loadLibrary("pclibrarycpp")
        }

    }
}

