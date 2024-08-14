package com.app.arcore

import android.content.Context
import android.content.pm.PackageManager
import android.widget.Toast
import androidx.fragment.app.FragmentActivity
import com.app.arcore.helpers.CameraPermissionHelper
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.sceneform.ArSceneView
import com.google.ar.sceneform.FrameTime

class ArCoreManager {
    private lateinit var arSceneView: ArSceneView
    private lateinit var session: Session
    private lateinit var config: Config

    fun initialize(context: Context, arSceneView: ArSceneView, activity: FragmentActivity) {
        this.arSceneView = arSceneView
        val isCameraPermissionGranted = CameraPermissionHelper.hasCameraPermission(activity)
        // Check if camera permission is granted
        if (!isCameraPermissionGranted){
            CameraPermissionHelper.requestCameraPermission(activity)
        } else {
            // If permission is already granted, proceed with session setup
            setupSession(context)
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
            }
        }
    }

    fun onResume() {
        arSceneView.resume()
        arSceneView.scene.addOnUpdateListener { frameTime: FrameTime? ->
            /**
             * Inside the listener, you can access the current AR frame.
             * In other words, the main processing of each frame occurs here.
             *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
             * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
             */
            val frame = arSceneView.arFrame
        }
    }

    fun onPause() {
        arSceneView.pause()
    }

    fun onDestroy() {
        arSceneView.destroy()
    }
}
