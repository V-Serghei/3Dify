package com.app.arcore

import android.content.Context
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.sceneform.ArSceneView
import com.google.ar.sceneform.FrameTime

class ArCoreManager {



    private lateinit var arSceneView: ArSceneView
    private lateinit var session: Session
    private lateinit var config: Config

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
        }
    }

    fun onPause() {
        arSceneView.pause()
    }

    fun onDestroy() {
        arSceneView.destroy()
    }
}
