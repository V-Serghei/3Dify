package com.app.arcore

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.app.threedify.databinding.ActivityArcoreBinding
import com.google.ar.core.Config
import com.google.ar.core.Session
import com.google.ar.sceneform.ArSceneView
import com.google.ar.sceneform.FrameTime


class ArcoreActivity : AppCompatActivity() {

    private lateinit var arSceneView: ArSceneView
    private lateinit var binding: ActivityArcoreBinding
    private lateinit var session: Session
    private lateinit var config: Config

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityArcoreBinding.inflate(layoutInflater)
        setContentView(binding.root)
        arSceneView = binding.arSceneView

        session = Session(this)

        config = Config(session)
        config.setUpdateMode(Config.UpdateMode.LATEST_CAMERA_IMAGE)
        session.configure(config)

        config.setPlaneFindingMode(Config.PlaneFindingMode.DISABLED)
        session.configure(config)


        arSceneView.setupSession(session)
    }

    override fun onResume() {
        super.onResume()
        arSceneView.scene.addOnUpdateListener { frameTime: FrameTime? ->
            // Здесь вы можете обрабатывать кадры AR сессии
            val frame = arSceneView.arFrame
        }

        arSceneView.resume()
    }

    override fun onPause() {
        super.onPause()
        arSceneView.pause()
    }

    override fun onDestroy() {
        super.onDestroy()
        arSceneView.destroy()
    }
}
