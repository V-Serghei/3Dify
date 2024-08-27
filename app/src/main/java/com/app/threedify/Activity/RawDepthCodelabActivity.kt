/*
 * Copyright 2021 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.app.threedify.Activity

import android.annotation.SuppressLint
import android.content.Intent
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.util.Log
import android.widget.Button
import android.widget.ImageButton
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.app.threedify.helpers.CameraPermissionHelper
import com.app.threedify.helpers.DisplayRotationHelper
import com.app.threedify.helpers.FullScreenHelper
import com.app.threedify.helpers.SnackbarHelper
import com.app.threedify.helpers.TrackingStateHelper
import com.app.arcore.common.rendering.BoxRenderer
import com.app.arcore.common.rendering.DepthRenderer
import com.app.threedify.rawdepth.DepthData
import com.app.threedify.R
import com.google.ar.core.ArCoreApk
import com.google.ar.core.ArCoreApk.InstallStatus
import com.google.ar.core.Config
import com.google.ar.core.Plane
import com.google.ar.core.Pose
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.codelab.common.rendering.BackgroundRenderer
import com.google.ar.core.exceptions.CameraNotAvailableException
import com.google.ar.core.exceptions.UnavailableApkTooOldException
import com.google.ar.core.exceptions.UnavailableArcoreNotInstalledException
import com.google.ar.core.exceptions.UnavailableDeviceNotCompatibleException
import com.google.ar.core.exceptions.UnavailableSdkTooOldException
import com.google.ar.core.exceptions.UnavailableUserDeclinedInstallationException
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.io.OutputStreamWriter
import java.nio.FloatBuffer
import javax.microedition.khronos.egl.EGLConfig
import javax.microedition.khronos.opengles.GL10
import kotlin.math.pow
import kotlin.math.sqrt


/**
 * This is a simple example that shows how to create an augmented reality (AR) application using the
 * ARCore Raw Depth API. The application will show 3D point-cloud data of the environment.
 */
class RawDepthCodelabActivity : AppCompatActivity(), GLSurfaceView.Renderer {
    // Rendering. The Renderers are created here, and initialized when the GL surface is created.
    private var surfaceView: GLSurfaceView? = null

    private var installRequested = false

    private var session: Session? = null
    private val messageSnackbarHelper: SnackbarHelper = SnackbarHelper()
    private var displayRotationHelper: DisplayRotationHelper? = null

    private val depthRenderer: DepthRenderer = DepthRenderer()
    private val backgroundRenderer: BackgroundRenderer = BackgroundRenderer()
    private val boxRenderer: BoxRenderer = BoxRenderer()

    private lateinit var toggleModeButtonCamera: Button
    private var currentMode = Mode.CAMERA

    private lateinit var togglePlanesFilteringButton: Button
    private lateinit var pointsToRenderSeekBar: SeekBar
    private lateinit var depthThresholdSeekBar: SeekBar
    private lateinit var confidenceThresholdSeekBar: SeekBar
    private lateinit var buttonIncreaseThreshold: SeekBar

    private lateinit var placeholderButton1: Button
    private lateinit var placeholderButton2: Button
    private lateinit var placeholderButton3: Button
    private lateinit var pointFixationButton: Button
    /**********************************
     * ********************************
     *Button fixed point cloud
     */
    private val savedPoints: MutableList<FloatBuffer> = mutableListOf()
    private var currentPoints: FloatBuffer? = null
    private var cameraPoseCurrent : Pose? = null
    private var cameraPose:Pose ? = null


    /**
     *Button fixed point cloud
     * **********************************
     */
    private var planesFiltringEnable = true

    @SuppressLint("MissingInflatedId")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_arcore)
        surfaceView = findViewById(R.id.surfaceview)
        displayRotationHelper = DisplayRotationHelper( /*context=*/this)

        // Set up renderer.
        surfaceView!!.preserveEGLContextOnPause = true
        surfaceView!!.setEGLContextClientVersion(2)
        surfaceView!!.setEGLConfigChooser(8, 8, 8, 8, 16, 0) // Alpha used for plane blending.
        surfaceView!!.setRenderer(this)
        surfaceView!!.renderMode = GLSurfaceView.RENDERMODE_CONTINUOUSLY
        surfaceView!!.setWillNotDraw(false)

        installRequested = false


        /**********************************
         * ********************************
         *Button test
         */
        toggleModeButtonCamera = findViewById(R.id.toggleModeButtonCamera)
        togglePlanesFilteringButton = findViewById(R.id.togglePlanesFilteringButton)
        pointsToRenderSeekBar = findViewById(R.id.pointsToRenderSeekBar)
        depthThresholdSeekBar = findViewById(R.id.depthThresholdSeekBar)
        confidenceThresholdSeekBar = findViewById(R.id.confidenceThresholdSeekBar)
        buttonIncreaseThreshold = findViewById(R.id.increaseThresholdSeekBar)

        placeholderButton1 = findViewById(R.id.testButton1)
        placeholderButton2 = findViewById(R.id.testButton2)
        placeholderButton3 = findViewById(R.id.testButton3)
        pointFixationButton = findViewById(R.id.point_fixation)

        toggleModeButtonCamera.setOnClickListener { toggleMode() }
        togglePlanesFilteringButton.setOnClickListener { togglePlanesFiltering() }

        /**
         * Below are three buttons that you can use as you wish
         *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
         * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
         */
        placeholderButton1.setOnClickListener{/**here will be called the method that you add here*/}
        placeholderButton2.setOnClickListener{/**here will be called the method that you add here*/}
        placeholderButton3.setOnClickListener{/**here will be called the method that you add here*/}

        pointFixationButton.setOnClickListener{fixatePoints()}

        pointsToRenderSeekBar.max = 100000
        pointsToRenderSeekBar.progress = DepthData.maxNumberOfPointsToRender.toInt()
        pointsToRenderSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.maxNumberOfPointsToRender = progress.toFloat()
                findViewById<TextView>(R.id.pointsToRenderValue).text = progress.toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        depthThresholdSeekBar.max = 3000
        depthThresholdSeekBar.progress = (DepthData.depthThreshold * 1000).toInt()
        depthThresholdSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.depthThreshold = progress / 1000.0f
                findViewById<TextView>(R.id.depthThresholdValue).text = (progress/1000.0f).toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        confidenceThresholdSeekBar.max = 100
        confidenceThresholdSeekBar.progress = (DepthData.confidenceThreshold * 100).toInt()
        confidenceThresholdSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.confidenceThreshold = progress / 100.0f
                findViewById<TextView>(R.id.confidenceThresholdValue).text = (progress/100.0f).toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })
        buttonIncreaseThreshold.max = 10
        buttonIncreaseThreshold.progress = (DepthData.planeDist * 100).toInt()
        buttonIncreaseThreshold.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.planeDist = progress / 100.0f
                findViewById<TextView>(R.id.increaseThresholdValue).text = (progress/100.0f).toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })



        val backButton = findViewById<ImageButton>(R.id.back_button)
        backButton.setOnClickListener {  finish() }

        toggleModeButtonCamera = findViewById(R.id.toggleModeButtonCamera)
        toggleModeButtonCamera.setOnClickListener {
            toggleMode()
        }

        /**********************************
         * ********************************
         * ********************************
         *Button test
         */

    }
    private fun fixatePoints() {
        val points = currentPoints ?: return
        val filteredPoints = filterInvalidPoints(points)
        if (filteredPoints.capacity() > 0) {
            savedPoints.add(filteredPoints)
            logPoints(filteredPoints)
            savePointsToFile(filteredPoints)
        }
    }
    private fun savePointsToFile(points: FloatBuffer) {
        val fileName = "coordinates_data.txt"
        val file = File(getExternalFilesDir(null), fileName)

        try {
            FileOutputStream(file, true).use { fos ->
                OutputStreamWriter(fos).use { writer ->
                    if (cameraPose != null) {
                        val tx = cameraPose?.tx()
                        val ty = cameraPose?.ty()
                        val tz = cameraPose?.tz()
                        writer.write("Device coordinates: x = $tx, y = $ty, z = $tz\n")
                    }

                    val pointCount = points.capacity() / 3
                    writer.write("Total Points: $pointCount\n")

                    for (i in 0 until pointCount) {
                        val pointX = points.get(i * 3)
                        val pointY = points.get(i * 3 + 1)
                        val pointZ = points.get(i * 3 + 2)
                        writer.write("Point $i: x = $pointX, y = $pointY, z = $pointZ\n")

                        if (cameraPose != null) {
                            val deviceX = cameraPose?.tx()
                            val deviceY = cameraPose?.ty()
                            val deviceZ = cameraPose?.tz()

                            val distance = sqrt(
                                (pointX - deviceX!!).toDouble().pow(2.0) +
                                        (pointY - deviceY!!).toDouble().pow(2.0) +
                                        (pointZ - deviceZ!!).toDouble().pow(2.0)
                            )

                            writer.write("Distance to point: $distance\n")
                        }
                    }

                    writer.write("\n")
                }
            }
        } catch (e: IOException) {
            Log.e(TAG, "Error saving points to file", e)
        }
    }
    private fun filterInvalidPoints(points: FloatBuffer): FloatBuffer {
        val validPoints = mutableListOf<Float>()
        val pointCount = points.capacity() / 3

        for (i in 0 until pointCount) {
            val x = points.get(i * 3)
            val y = points.get(i * 3 + 1)
            val z = points.get(i * 3 + 2)

            if (x != 0.0f || y != 0.0f || z != 0.0f) {
                validPoints.add(x)
                validPoints.add(y)
                validPoints.add(z)
            }
        }

        val validPointsBuffer = FloatBuffer.allocate(validPoints.size)
        validPointsBuffer.put(validPoints.toFloatArray())
        validPointsBuffer.rewind()
        return validPointsBuffer
    }
    private fun logPoints(points: FloatBuffer) {
        cameraPose = cameraPoseCurrent
       try {
            if (cameraPose != null) {
                val tx = cameraPose?.tx()
                val ty = cameraPose?.ty()
                val tz = cameraPose?.tz()
                println("Device coordinates: x = $tx, y = $ty, z = $tz")
            }


            val pointCount = points.capacity() / 3
            println("Total Points: $pointCount")

            for (i in 0 until pointCount) {
                val pointX = points.get(i * 3)
                val pointY = points.get(i * 3 + 1)
                val pointZ = points.get(i * 3 + 2)
                println("Point $i: x = $pointX, y = $pointY, z = $pointZ")

                if (cameraPose != null) {
                    val deviceX = cameraPose?.tx()
                    val deviceY = cameraPose?.ty()
                    val deviceZ = cameraPose?.tz()


                        val distance = sqrt(
                            (pointX - deviceX!!).toDouble().pow(2.0) +
                                    (pointY - deviceY!!).toDouble().pow(2.0) +
                                    (pointZ - deviceZ!!).toDouble().pow(2.0)
                        )

                        println("Distance to point: $distance")
                }

            }

        }catch (ex :NullPointerException){
           messageSnackbarHelper.showError(this, "Cannot get phone coordinates")
           Log.e(TAG, "Exception creating session", ex)
        }
    }
    private fun togglePlanesFiltering() {
        planesFiltringEnable = !planesFiltringEnable
        togglePlanesFilteringButton.text = if (planesFiltringEnable) "Disable Planes Filtering" else "Enable Planes Filtering"
    }

    private fun returnHomeMenu() {

        startActivity(Intent(this, MainActivity::class.java))
        finish()
    }

    override fun onResume() {
        super.onResume()

        if (session == null) {
            var exception: Exception? = null
            var message: String? = null
            try {
                when (ArCoreApk.getInstance().requestInstall(this, !installRequested)) {
                    InstallStatus.INSTALL_REQUESTED -> {
                        installRequested = true
                        return
                    }

                    InstallStatus.INSTALLED -> {}
                }
                // ARCore requires camera permissions to operate. If we did not yet obtain runtime
                // permission on Android M and above, now is a good time to ask the user for it.
                if (!CameraPermissionHelper.hasCameraPermission(this)) {
                    CameraPermissionHelper.requestCameraPermission(this)
                    return
                }

                // Creates the ARCore session.
                session = Session( /* context= */this)
                if (!session!!.isDepthModeSupported(Config.DepthMode.RAW_DEPTH_ONLY)) {
                    message = (
                            "This device does not support the ARCore Raw Depth API. See"
                                    + " https://developers.google.com/ar/discover/supported-devices.")
                }
            } catch (e: UnavailableArcoreNotInstalledException) {
                message = "Please install ARCore"
                exception = e
            } catch (e: UnavailableUserDeclinedInstallationException) {
                message = "Please install ARCore"
                exception = e
            } catch (e: UnavailableApkTooOldException) {
                message = "Please update ARCore"
                exception = e
            } catch (e: UnavailableSdkTooOldException) {
                message = "Please update this app"
                exception = e
            } catch (e: UnavailableDeviceNotCompatibleException) {
                message = "This device does not support AR"
                exception = e
            } catch (e: Exception) {
                message = "Failed to create AR session"
                exception = e
            }

            if (message != null) {
                messageSnackbarHelper.showError(this, message)
                Log.e(TAG, "Exception creating session", exception)
                return
            }
        }

        try {
            // Enable raw depth estimation and auto focus mode while ARCore is running.
            val config = session!!.config
            config.setDepthMode(Config.DepthMode.RAW_DEPTH_ONLY)
            config.setFocusMode(Config.FocusMode.AUTO)
            session!!.configure(config)
            session!!.resume()
        } catch (e: CameraNotAvailableException) {
            messageSnackbarHelper.showError(this, "Camera not available. Try restarting the app.")
            session = null
            return
        }

        // Note that order matters - see the note in onPause(), the reverse applies here.
        surfaceView!!.onResume()
        displayRotationHelper?.onResume()
        if (currentMode == Mode.RAW_DEPTH) {
            messageSnackbarHelper.showMessage(this, "Waiting for depth data...")
        }
    }

    override fun onPause() {
        super.onPause()
        if (session != null) {
            // Note that the order matters - GLSurfaceView is paused first so that it does not try
            // to query the session. If Session is paused before GLSurfaceView, GLSurfaceView may
            // still call session.update() and get a SessionPausedException.
            displayRotationHelper?.onPause()
            surfaceView!!.onPause()
            session!!.pause()
        }
    }

    @Override
    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<String>,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        if (!CameraPermissionHelper.hasCameraPermission(this)) {
            Toast.makeText(
                this, "Camera permission is needed to run this application",
                Toast.LENGTH_LONG
            ).show()
            if (!CameraPermissionHelper.shouldShowRequestPermissionRationale(this)) {
                // Permission denied with checking "Do not ask again".
                CameraPermissionHelper.launchPermissionSettings(this)
            }
            finish()
        }
    }


    override fun onWindowFocusChanged(hasFocus: Boolean) {
        super.onWindowFocusChanged(hasFocus)
        FullScreenHelper.setFullScreenOnWindowFocusChanged(this, hasFocus)
    }

    override fun onSurfaceCreated(gl: GL10, config: EGLConfig) {
        GLES20.glClearColor(0.1f, 0.1f, 0.1f, 1.0f)

        // Prepare the rendering objects. This involves reading shaders, so may throw an IOException.
        try {
            // Create the texture and pass it to ARCore session to be filled during update().
            backgroundRenderer.createOnGlThread( /*context=*/this)
            depthRenderer.createOnGlThread( /*context=*/this)
            boxRenderer.createOnGlThread( /*context=*/this)
        } catch (e: IOException) {
            Log.e(TAG, "Failed to read an asset file", e)
        }
    }

    override fun onSurfaceChanged(gl: GL10, width: Int, height: Int) {
        displayRotationHelper?.onSurfaceChanged(width, height)
        GLES20.glViewport(0, 0, width, height)
    }

    override fun onDrawFrame(gl: GL10) {
        // Clear screen to notify driver it should not load any pixels from previous frame.
        GLES20.glClear(GLES20.GL_COLOR_BUFFER_BIT or GLES20.GL_DEPTH_BUFFER_BIT)

        if (session == null) {
            return
        }
        // Notify ARCore session that the view size changed so that the perspective matrix and
        // the video background can be properly adjusted.
        displayRotationHelper?.updateSessionIfNeeded(session!!)

        try {
            session!!.setCameraTextureName(backgroundRenderer.textureId)

            // Obtain the current frame from ARSession. When the configuration is set to
            // UpdateMode.BLOCKING (it is by default), this will throttle the rendering to the
            // camera framerate.
            val frame = session!!.update()
            val camera = frame.camera

            // If frame is ready, render camera preview image to the GL surface.
            backgroundRenderer.draw(frame)

            //If the raw depth mode chosen, visualize depth points
            if (currentMode == Mode.RAW_DEPTH) {
                // Retrieve the depth data for this frame.
                val points: FloatBuffer =
                    DepthData.create(frame, session!!.createAnchor(camera.pose)) ?: return

                if (points != null) {
                    currentPoints = points
                    cameraPoseCurrent = camera.pose

                    // Filters the depth data.
                    if (planesFiltringEnable) {
                        DepthData.filterUsingPlanes(
                            points,
                            session!!.getAllTrackables(Plane::class.java)
                        )
                    }
                    for (savedPointBuffer in savedPoints) {
                        depthRenderer.update(savedPointBuffer)
                        depthRenderer.draw(camera)
                    }
                    // Visualize depth points.
                    depthRenderer.update(points)
                    depthRenderer.draw(camera)
                }

                // If not tracking, show tracking failure reason instead.
                if (camera.trackingState == TrackingState.PAUSED) {
                    messageSnackbarHelper.showMessage(
                        this, TrackingStateHelper.getTrackingFailureReasonString(camera)
                    )
                    return
                }

                //If you want drawn boxes uncomment
//            // Draw boxes around clusters of points.
//            val clusteringHelper: PointClusteringHelper = PointClusteringHelper(points)
//            val clusters: List<AABB> = clusteringHelper.findClusters()
//            for (aabb in clusters) {
//                boxRenderer.draw(aabb, camera)
//            }
            }
        } catch (t: Throwable) {
            // Avoid crashing the application due to unhandled exceptions.
            Log.e(TAG, "Exception on the OpenGL thread", t)
        }
    }

    companion object {
        private val TAG: String = RawDepthCodelabActivity::class.java.simpleName
    }
    //When camera mode changed, change label and visualisation
    private fun toggleMode() {
        currentMode = when (currentMode) {
            Mode.CAMERA -> Mode.RAW_DEPTH
            Mode.RAW_DEPTH -> Mode.CAMERA
        }
        toggleModeButtonCamera.text = when (currentMode) {
            Mode.CAMERA -> getString(R.string.toggleButtonCameraText)
            Mode.RAW_DEPTH -> getString(R.string.toggleButtonRawDepthText)
        }
    }

    enum class Mode {
        CAMERA,
        RAW_DEPTH
    }
}