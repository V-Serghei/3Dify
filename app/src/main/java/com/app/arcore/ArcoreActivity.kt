package com.app.arcore

import android.os.Bundle
import androidx.appcompat.app.AppCompatActivity
import com.app.threedify.databinding.ActivityArcoreBinding

class ArcoreActivity : AppCompatActivity() {

    private lateinit var binding: ActivityArcoreBinding
    private val arCoreManager = ArCoreManager()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityArcoreBinding.inflate(layoutInflater)
        setContentView(binding.root)

        arCoreManager.initialize(this, binding.arSceneView)
    }

    override fun onResume() {
        super.onResume()
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
}
