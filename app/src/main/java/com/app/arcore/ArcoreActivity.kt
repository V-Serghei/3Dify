package com.app.arcore

import android.annotation.SuppressLint
import android.content.ContentValues.TAG
import android.os.Bundle
import android.util.Log
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.app.arcore.helpers.CameraPermissionHelper
import com.app.threedify.R
import com.app.threedify.R.id.surfaceView
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException

class ArcoreActivity : AppCompatActivity() {
    private var mUserRequestedInstall = true
    private var mSession: Session? = null
    private lateinit var surfaceView: SurfaceView
    //private val myTextView: TextView = findViewById(R.id.my_text_view)

    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_arcore)
        surfaceView = findViewById(R.id.surfaceView)
    }

    @SuppressLint("SetTextI18n")
    override fun onResume() {
        super.onResume()
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            CameraPermissionHelper.requestCameraPermission(this)
            //myTextView.text = "1"
            return
        }
        try {
            if (mSession == null) {
                when (ArCoreApk.getInstance().requestInstall(this, mUserRequestedInstall)) {
                    ArCoreApk.InstallStatus.INSTALLED -> {
                        mSession = Session(this)
                        createSession()
                    }

                    ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                        mUserRequestedInstall = false
                        //myTextView.text = "2"
                        return
                    }
                }
            }
        } catch (e: UnavailableUserDeclinedInstallationException) {
            Toast.makeText(this, "TODO: handle exception " + e, Toast.LENGTH_LONG).show()
            //myTextView.text = "3"
            return
        }
        try {
            mSession?.resume()
            surfaceView.holder.addCallback(object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) {
                    mSession?.setCameraTextureName(holder.surface.hashCode())
                }

                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {}

                override fun surfaceDestroyed(holder: SurfaceHolder) {}
            })
        } catch (e: CameraNotAvailableException) {
            Log.e(TAG, "Камера не доступна", e)
            //myTextView.text = "4"
            mSession = null
            return
        }
    }

    @SuppressLint("SetTextI18n")
    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            Toast.makeText(this, "Camera permission is needed to run this application", Toast.LENGTH_LONG)
                .show()
            //myTextView.text = "5"
            if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
                CameraPermissionHelper.launchPermissionSettings(this)
               // myTextView.text = "6"
            }
            finish()
        }
    }

    override fun onPause() {
        super.onPause()
        mSession?.pause()
    }

    override fun onDestroy() {
        super.onDestroy()
        mSession?.close()
        mSession = null
    }

    // Verify that ARCore is installed and using the current version.
    fun isARCoreSupportedAndUpToDate(): Boolean {
        return when (ArCoreApk.getInstance().checkAvailability(this)) {
            ArCoreApk.Availability.SUPPORTED_INSTALLED -> true
            ArCoreApk.Availability.SUPPORTED_APK_TOO_OLD, ArCoreApk.Availability.SUPPORTED_NOT_INSTALLED -> {
                try {
                    // Request ARCore installation or update if needed.
                    when (ArCoreApk.getInstance().requestInstall(this, true)) {
                        ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                            Log.i(TAG, "ARCore installation requested.")
                            false
                        }
                        ArCoreApk.InstallStatus.INSTALLED -> true
                    }
                } catch (e: UnavailableException) {
                    Log.e(TAG, "ARCore not installed", e)
                    false
                }
            }
            ArCoreApk.Availability.UNSUPPORTED_DEVICE_NOT_CAPABLE ->
                // This device is not supported for AR.
                false

            ArCoreApk.Availability.UNKNOWN_CHECKING -> {
                // ARCore is checking the availability with a remote query.
                // This function should be called again after waiting 200 ms to determine the query result.
                false
            }
            ArCoreApk.Availability.UNKNOWN_ERROR, ArCoreApk.Availability.UNKNOWN_TIMED_OUT -> {
                // There was an error checking for AR availability. This may be due to the device being offline.
                // Handle the error appropriately.
                false
            }
        }
    }
    private fun createSession() {
        // Create a new ARCore session.
        mSession = Session(this)

        // Create a session config.
        val config = Config(mSession)

        // Do feature-specific operations here, such as enabling depth or turning on
        // support for Augmented Faces.

        // Configure the session.
        mSession!!.configure(config)
    }

    fun maybeEnableArButton() {
        ArCoreApk.getInstance().checkAvailabilityAsync(this) { availability ->
            if (availability.isSupported) {
                //
            } else {
                //
            }
        }
    }
}