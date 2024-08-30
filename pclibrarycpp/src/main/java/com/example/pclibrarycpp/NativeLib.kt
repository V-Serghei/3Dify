package com.example.pclibrarycpp

class NativeLib {

    /**
     * A native method that is implemented by the 'pclibrarycpp' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String

    companion object {
        // Used to load the 'pclibrarycpp' library on application startup.
        init {
            System.loadLibrary("pclibrarycpp")
        }
    }
}