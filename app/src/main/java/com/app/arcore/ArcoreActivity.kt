package com.app.arcore

import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.content.ContentValues.TAG
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.app.arcore.helpers.CameraPermissionHelper
import com.app.threedify.R
import com.app.threedify.databinding.ActivityArcoreBinding
import com.google.ar.core.ArCoreApk
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException

class ArcoreActivity : AppCompatActivity() {

    private lateinit var binding: ActivityArcoreBinding
    private val arCoreManager = ArCoreManager()
    private var mUserRequestedInstall = true
    private lateinit var surfaceView: SurfaceView

    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityArcoreBinding.inflate(layoutInflater)
        setContentView(binding.root)
        surfaceView = findViewById(R.id.ar_scene_view)
        arCoreManager.initialize(this, binding.arSceneView, this)
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
            if (arCoreManager.session == null) {
                when (ArCoreApk.getInstance().requestInstall(this, mUserRequestedInstall)) {
                    ArCoreApk.InstallStatus.INSTALLED -> {
                        arCoreManager.onResume()
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
            arCoreManager.session.resume()
            surfaceView.holder.addCallback(object : SurfaceHolder.Callback {
                override fun surfaceCreated(holder: SurfaceHolder) {
                    arCoreManager.session.setCameraTextureName(holder.surface.hashCode())
                }

                override fun surfaceChanged(holder: SurfaceHolder, format: Int, width: Int, height: Int) {}

                override fun surfaceDestroyed(holder: SurfaceHolder) {}
            })
        } catch (e: CameraNotAvailableException) {
            Log.e(TAG, "Camera is not available", e)
            //myTextView.text = "4"
            return
        }
        arCoreManager.onResume()
    }

    override fun onPause() {
        super.onPause()
        arCoreManager.onPause()
    }

    override fun onDestroy() {
        super.onDestroy()
        arCoreManager.onDestroy()
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
