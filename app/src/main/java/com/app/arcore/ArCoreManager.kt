package com.app.arcore

import android.content.ContentValues.TAG
import android.content.Context
import android.content.pm.PackageManager
import android.util.Log
import android.widget.Toast
import androidx.fragment.app.FragmentActivity
import com.app.arcore.helpers.CameraPermissionHelper
import com.google.ar.core.ArCoreApk
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.PointCloud
import com.google.ar.core.Session
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableException
import com.google.ar.sceneform.ArSceneView
import com.google.ar.sceneform.FrameTime

class ArCoreManager {
    private lateinit var arSceneView: ArSceneView
    lateinit var session: Session
    private lateinit var config: Config
    private var pointCloudEnabled = false
    private var isSessionReady = false
    fun initialize(context: Context, arSceneView: ArSceneView, activity: FragmentActivity) {
        this.arSceneView = arSceneView
        // Check if camera permission is granted
        if (!CameraPermissionHelper.hasCameraPermission(activity)) {
            // Request camera permission if not granted
            CameraPermissionHelper.requestCameraPermission(activity)
        } else {
            // If permission is already granted, check ARCore support and proceed
            maybeSetupSession(context, activity)
        }
    }
    private fun maybeSetupSession(context: Context, activity: FragmentActivity) {
        if (isARCoreSupportedAndUpToDate(context, activity)) {
            setupSession(context)
            isSessionReady = true // Mark the session as ready
        } else {
            // Handle absence of ARCore
            Toast.makeText(context, "ARCore is not supported or up-to-date on this device.", Toast.LENGTH_LONG).show()
        }
    }
    // Method to handle session setup after permission is granted
    private fun setupSession(context: Context) {
        session = Session(context)

        config = Config(session).apply {
            /**
             * Here, the main configuration of the ARCore session
             * and its functionality is set up
             *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
             * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
             */
            focusMode = Config.FocusMode.AUTO
            updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            planeFindingMode = Config.PlaneFindingMode.DISABLED
        }
        session.configure(config)
        arSceneView.setupSession(session)
    }
    fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<String>, grantResults: IntArray
    ) {
        if (requestCode == CameraPermissionHelper.CAMERA_PERMISSION_REQUEST_CODE) {
            if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                // If permission is granted, set up the ARCore session
                setupSession(arSceneView.context)
            } else {
                // Permission denied, show a message
                Toast.makeText(arSceneView.context, "Camera permission is required for AR", Toast.LENGTH_LONG).show()
                isSessionReady = false
            }
        }
    }

    fun onResume(context: Context, activity: FragmentActivity) {
        if (!isSessionReady) {
            return // Do not resume if the session is not ready
        }
        if (session == null) {
            try {
                if (isARCoreSupportedAndUpToDate(context, activity)) {
                    setupSession(context)
                } else {
                    return
                }
            } catch (e: UnavailableException) {
                Toast.makeText(context, "ARCore is not available.", Toast.LENGTH_LONG).show()
                Log.e(TAG, "ARCore is not available", e)
                return
            }
        }
        try {
        arSceneView.resume()
        arSceneView.scene.addOnUpdateListener { frameTime: FrameTime? ->
            /**
             * Inside the listener, you can access the current AR frame.
             * In other words, the main processing of each frame occurs here.
             *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
             * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
             */
            val frame = arSceneView.arFrame
            if (pointCloudEnabled && frame != null) {
                displayPointCloud(frame)
            }

        }
        } catch (e: CameraNotAvailableException) {
            Toast.makeText(context, "Camera is not available", Toast.LENGTH_LONG).show()

            Log.e(TAG, "Camera is not available", e)
                return
        }
    }

    private fun displayPointCloud(frame: Frame) {
        val pointCloud = frame.acquirePointCloud()
        if (pointCloud != null) {

            clearPreviousPointCloud()

            renderPointCloud(pointCloud)

            pointCloud.release()
        }
    }
    private fun clearPreviousPointCloud() {

    }

    fun togglePointCloud() {
        pointCloudEnabled = !pointCloudEnabled
    }

    private fun renderPointCloud(pointCloud: PointCloud) {

    }

    fun onPause() {
        arSceneView.pause()
    }

    fun onDestroy() {
        arSceneView.destroy()
    }
    // Verify that ARCore is installed and using the current version.
    private fun isARCoreSupportedAndUpToDate(context: Context,activity: FragmentActivity): Boolean {
        return when (ArCoreApk.getInstance().checkAvailability(context)) {
            ArCoreApk.Availability.SUPPORTED_INSTALLED -> true
            ArCoreApk.Availability.SUPPORTED_APK_TOO_OLD, ArCoreApk.Availability.SUPPORTED_NOT_INSTALLED -> {
                try {
                    // Request ARCore installation or update if needed.
                    when (ArCoreApk.getInstance().requestInstall(activity, true)) {
                        ArCoreApk.InstallStatus.INSTALL_REQUESTED -> {
                            Log.i(TAG, "ARCore installation requested.")
                            Toast.makeText(context, "ARCore installation requested.", Toast.LENGTH_LONG).show()
                            false
                        }
                        ArCoreApk.InstallStatus.INSTALLED ->{
                            setupSession(context)

                            true
                        }
                    }
                } catch (e: UnavailableException) {
                    Log.e(TAG, "ARCore not installed", e)
                    Toast.makeText(context, "ARCore not installed", Toast.LENGTH_LONG).show()

                    false
                }
            }
            ArCoreApk.Availability.UNSUPPORTED_DEVICE_NOT_CAPABLE ->{
                // This device is not supported for AR.
                Toast.makeText(context, "This device is not supported for AR.", Toast.LENGTH_LONG).show()
                false
            }


            ArCoreApk.Availability.UNKNOWN_CHECKING -> {
                Toast.makeText(context, "ARCore is checking the availability with a remote query.", Toast.LENGTH_LONG).show()
                // ARCore is checking the availability with a remote query.
                // This function should be called again after waiting 200 ms to determine the query result.
                false
            }
            ArCoreApk.Availability.UNKNOWN_ERROR, ArCoreApk.Availability.UNKNOWN_TIMED_OUT -> {
                Toast.makeText(context, "There was an error checking for AR availability. " +
                        "This may be due to the device being offline.", Toast.LENGTH_LONG).show()
                // There was an error checking for AR availability. This may be due to the device being offline.
                // Handle the error appropriately.
                false
            }
        }
    }
    fun maybeEnableArButton(context: Context) {
        ArCoreApk.getInstance().checkAvailabilityAsync(context) { availability ->
            if (availability.isSupported) {
                //
            } else {
                //
            }
        }
    }
}
