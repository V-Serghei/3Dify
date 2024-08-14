package com.app.arcore

import android.content.Context
import com.google.ar.core.Config
import com.google.ar.core.Frame
import com.google.ar.core.PointCloud
import com.google.ar.core.Session
import com.google.ar.sceneform.ArSceneView
import com.google.ar.sceneform.FrameTime

class ArCoreManager {



    private lateinit var arSceneView: ArSceneView
    private lateinit var session: Session
    private lateinit var config: Config
    private var pointCloudEnabled = false

    fun initialize(context: Context, arSceneView: ArSceneView) {
        this.arSceneView = arSceneView

        session = Session(context)

        config = Config(session).apply {
            /**
             * Here, the main configuration of the ARCore session
             * and its functionality is set up
             *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
             * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
             */
            updateMode = Config.UpdateMode.LATEST_CAMERA_IMAGE
            planeFindingMode = Config.PlaneFindingMode.DISABLED

        }
        session.configure(config)
        arSceneView.setupSession(session)
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
            if (pointCloudEnabled && frame != null) {
                displayPointCloud(frame)
            }
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
}
