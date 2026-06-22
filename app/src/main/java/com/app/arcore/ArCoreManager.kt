package com.app.arcore

import android.content.Context
import androidx.fragment.app.FragmentActivity
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.PointCloud
import io.github.sceneview.ar.ArSceneView

class ArCoreManager {
    private lateinit var arSceneView: ArSceneView
    private var pointCloudEnabled = false

    fun initialize(context: Context, arSceneView: ArSceneView, activity: FragmentActivity) {
        this.arSceneView = arSceneView

        // SceneView manages the ARCore session and camera permissions internally.
        // Configure session options via the callback below.
        arSceneView.configureSession = { _, config ->
            config.focusMode = Config.FocusMode.AUTO
            config.updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            config.planeFindingMode = Config.PlaneFindingMode.DISABLED
        }

        // Called on every AR frame — main per-frame processing goes here
        arSceneView.onArFrame = { arFrame ->
            if (pointCloudEnabled) {
                displayPointCloud(arFrame.frame)
            }
        }
    }

    fun onResume(context: Context, activity: FragmentActivity) {
        arSceneView.onResume()
    }

    fun onPause() {
        arSceneView.onPause()
    }

    fun onDestroy() {
        arSceneView.destroy()
    }

    fun togglePointCloud() {
        pointCloudEnabled = !pointCloudEnabled
    }

    private fun displayPointCloud(frame: Frame) {
        val pointCloud: PointCloud? = frame.acquirePointCloud()
        if (pointCloud != null) {
            clearPreviousPointCloud()
            renderPointCloud(pointCloud)
            pointCloud.release()
        }
    }

    private fun clearPreviousPointCloud() {}

    private fun renderPointCloud(pointCloud: PointCloud) {}
}
