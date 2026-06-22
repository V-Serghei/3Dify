package com.app.arcore

import android.content.Context
import android.view.View
import androidx.fragment.app.FragmentActivity
import com.google.ar.core.Frame
import com.google.ar.core.PointCloud

class ArCoreManager {
    private var pointCloudEnabled = false

    fun initialize(context: Context, arView: View, activity: FragmentActivity) {
        // SceneView ArSceneView integration goes here.
        // Cast arView to ArSceneView and configure session + frame listener when ready.
    }

    fun onResume(context: Context, activity: FragmentActivity) {}

    fun onPause() {}

    fun onDestroy() {}

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
