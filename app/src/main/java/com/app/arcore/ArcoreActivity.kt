package com.app.arcore

import android.annotation.SuppressLint
import android.content.ContentValues.TAG
import android.os.Bundle
import android.util.Log
import android.view.SurfaceHolder
import android.view.SurfaceView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.app.arcore.common.helpers.CameraPermissionHelper
import com.app.threedify.R
import com.app.threedify.databinding.ActivityArcoreBinding
import com.google.ar.core.ArCoreApk
import com.google.ar.core.exceptions.CameraNotAvailableException
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
        //arCoreManager.initialize(this, binding.arSceneView, this)
    }
    @SuppressLint("SetTextI18n")
    override fun onResume() {
        super.onResume()
        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            CameraPermissionHelper.requestCameraPermission(this)
            return
        }
        try {
            if (arCoreManager.session == null) {
                when (ArCoreApk.getInstance().requestInstall(this, mUserRequestedInstall)) {
                    ArCoreApk.InstallStatus.INSTALLED -> {
                        arCoreManager.onResume(this,this)
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
            return
        }
        arCoreManager.onResume(this,this)
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
