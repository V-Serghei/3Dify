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
import android.content.ContentValues
import android.content.Context
import android.content.Intent
import android.net.Uri
import android.opengl.GLES20
import android.opengl.GLSurfaceView
import android.os.Bundle
import android.os.Environment
import android.provider.MediaStore
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.ImageButton
import android.widget.SeekBar
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import com.app.threedify.helpers.CameraPermissionHelper
import com.app.threedify.helpers.DisplayRotationHelper
import com.app.threedify.helpers.FullScreenHelper
import com.app.threedify.helpers.ScanPointAccumulator
import com.app.threedify.helpers.SnackbarHelper
import com.app.threedify.helpers.TrackingStateHelper
import com.app.arcore.common.rendering.BoxRenderer
import com.app.arcore.common.rendering.CubeRenderer
import com.app.arcore.common.rendering.DepthRenderer
import com.app.arcore.common.rendering.SphereRenderer
import com.app.threedify.rawdepth.DepthData
import com.app.threedify.R
import com.app.threedify.rawdepth.DepthData2
import com.example.nativelib.Model3DCreator
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
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import java.util.UUID
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
    private val cubeRenderer: CubeRenderer = CubeRenderer()
    private val sphereRenderer: SphereRenderer = SphereRenderer()
    private val backgroundRenderer: BackgroundRenderer = BackgroundRenderer()
    private val boxRenderer: BoxRenderer = BoxRenderer()

    private lateinit var toggleModeButtonCamera: Button
    private var currentMode = Mode.CAMERA

    private lateinit var togglePlanesFilteringButton: Button
    private lateinit var pointsToRenderSeekBar: SeekBar
    private lateinit var depthThresholdSeekBar: SeekBar
    private lateinit var confidenceThresholdSeekBar: SeekBar
    private lateinit var buttonIncreaseThreshold: SeekBar
    private lateinit var amountOfPointsSeekBar: SeekBar
    private lateinit var minDepthThresholdSeekBar: SeekBar
    private lateinit var edgeFilterSeekBar: SeekBar
    private lateinit var voxelSizeSeekBar: SeekBar
    private lateinit var stabilitySeekBar: SeekBar

    private lateinit var startScanButton: Button
    private lateinit var settingsToggleButton: Button
    private lateinit var testDataButton: Button
    private lateinit var createModelButton: Button
    private lateinit var scanResetButton: Button
    private lateinit var settingsPanel: View
    private lateinit var pointsAmountTextView: TextView

    /**********************************
     * ********************************
     *The main keeper of points before sending to the native library.
     *********************************/
    private lateinit var pointArrays: Array<FloatArray>
    //Native lib class
    private val model3DCreator = Model3DCreator()

    /**********************************
     * ********************************
     *Button fixed point cloud
     */
    private val savedPoints: MutableList<FloatBuffer> = mutableListOf()
    private var currentPoints: FloatBuffer? = null
    private var cameraPoseCurrent : Pose? = null
    private var cameraPose:Pose ? = null
    private val scanPointAccumulator = ScanPointAccumulator()


    /**
     *Button fixed point cloud
     * **********************************
     */



    //------------------- maxPointsAmount

    private var maxPointsAmount = 0
    private var pointsCounterM : Int = 0

    private var renderPointBuffer: FloatBuffer = FloatBuffer.allocate(0)

    private var needToBeProcessed : Boolean = false

    //-------------------



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
        amountOfPointsSeekBar = findViewById(R.id.amountOfPointsSeekBar)
        minDepthThresholdSeekBar = findViewById(R.id.minDepthThresholdSeekBar)
        edgeFilterSeekBar = findViewById(R.id.edgeFilterSeekBar)
        voxelSizeSeekBar = findViewById(R.id.voxelSizeSeekBar)
        stabilitySeekBar = findViewById(R.id.stabilitySeekBar)

        startScanButton = findViewById(R.id.testButton1)
        settingsToggleButton = findViewById(R.id.settingsToggleButton)
        testDataButton = findViewById(R.id.testButton2)
        createModelButton = findViewById(R.id.testButton3)
        scanResetButton = findViewById(R.id.point_reset)
        settingsPanel = findViewById(R.id.settingsPanel)
        pointsAmountTextView = findViewById(R.id.points_amount)

        toggleModeButtonCamera.setOnClickListener { toggleMode() }
        togglePlanesFilteringButton.setOnClickListener { togglePlanesFiltering() }
        settingsToggleButton.setOnClickListener { toggleSettingsPanel() }

        /**
         * Below are three buttons that you can use as you wish
         *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
         * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
         */
        startScanButton.setOnClickListener{
            if(!needToBeProcessed){
                startScan()
            }else{
                stopScan()
            }
        }

        /**
         * ********************************
         * ********************************
         * Saving points from the file point.txt
         * Generation of a model based on this data.
         */
        testDataButton.setOnClickListener{
            pointArrays = loadPointsFromAssets(this, "point.txt")
            //val fileName = generateFileName("model_txt","obj")
            //saveModelToUri(fileName)


            var buffer : FloatBuffer = FloatBuffer.allocate(50000)

            this.assets.open("sphere.txt").bufferedReader().use { reader ->
                reader.forEachLine { line ->
                    val parts = line.split(" ")
                    if (parts.size == 3) {
                        val x = parts[0].toFloat()
                        val y = parts[1].toFloat()
                        val z = parts[2].toFloat()
                        buffer.put(x)
                        buffer.put(y)
                        buffer.put(z)
                    }
                }
            }


            //buffer.put(0.0f)
            //buffer.put(0.0f)
            //buffer.put(0.0f)
//
            //buffer.put(0.0f)
            //buffer.put(1.0f)
            //buffer.put(0.0f)
//
            //buffer.put(1.0f)
            //buffer.put(1.0f)
            //buffer.put(0.0f)
//
            //buffer.put(1.0f)
            //buffer.put(0.0f)
            //buffer.put(0.0f)
//
            //buffer.put(0.0f)
            //buffer.put(0.0f)
            //buffer.put(1.0f)
//
            //buffer.put(0.0f)
            //buffer.put(1.0f)
            //buffer.put(1.0f)
//
            //buffer.put(1.0f)
            //buffer.put(1.0f)
            //buffer.put(1.0f)

            //buffer.put(1.0f)
            //buffer.put(0.0f)
            //buffer.put(1.0f)


            val cordsTxt: StringBuilder = StringBuilder("")

            for (i in 0 until 4066) {
                val x = buffer.get(i * 3)
                val y = buffer.get(i * 3 + 1)
                val z = buffer.get(i * 3 + 2)

                cordsTxt.append("$x $y $z\n")
            }

            val externalFile = File(getExternalFilesDir(null), "cords.txt")
            externalFile.appendText(cordsTxt.toString())

            scanPointAccumulator.clear()
            val testPoints = xyzBufferToRenderBuffer(buffer, 4066)
            scanPointAccumulator.add(testPoints)
            scanPointAccumulator.add(testPoints)
            currentPoints = scanPointAccumulator.toRenderBuffer()
            fixatePoints()
            savePointsToPCDFile()
            preparePointsForModel()
        }

        createModelButton.setOnClickListener {
            stopScan()
            currentPoints = scanPointAccumulator.toRenderBuffer()
            fixatePoints()
            savePointsToPCDFile()
            preparePointsForModel()
        }

        scanResetButton.setOnClickListener{
            stopScan()
            clearAccumulatedScan()
        }

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
                scanPointAccumulator.minConfidence = DepthData.confidenceThreshold
                findViewById<TextView>(R.id.confidenceThresholdValue).text = (progress/100.0f).toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })
        scanPointAccumulator.minConfidence = DepthData.confidenceThreshold
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

        minDepthThresholdSeekBar.max = 100
        minDepthThresholdSeekBar.progress = (DepthData.minDepthThreshold * 100).toInt()
        minDepthThresholdSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.minDepthThreshold = (progress.coerceAtLeast(5)) / 100.0f
                findViewById<TextView>(R.id.minDepthThresholdValue).text = "%.2f".format(DepthData.minDepthThreshold)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        edgeFilterSeekBar.max = 20
        edgeFilterSeekBar.progress = (DepthData.edgeFilterDeltaMeters * 100).toInt()
        edgeFilterSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                DepthData.edgeFilterDeltaMeters = progress / 100.0f
                findViewById<TextView>(R.id.edgeFilterValue).text = "%.2f".format(DepthData.edgeFilterDeltaMeters)
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        voxelSizeSeekBar.max = 30
        voxelSizeSeekBar.progress = (scanPointAccumulator.voxelSizeMeters * 1000).toInt()
        voxelSizeSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                scanPointAccumulator.voxelSizeMeters = progress.coerceAtLeast(5) / 1000.0f
                findViewById<TextView>(R.id.voxelSizeValue).text = "%.3f".format(scanPointAccumulator.voxelSizeMeters)
                if (fromUser) {
                    clearAccumulatedScan()
                }
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        stabilitySeekBar.max = 6
        stabilitySeekBar.progress = scanPointAccumulator.minObservationsForModel
        stabilitySeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                scanPointAccumulator.minObservationsForModel = progress.coerceAtLeast(1)
                findViewById<TextView>(R.id.stabilityValue).text = scanPointAccumulator.minObservationsForModel.toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })

        amountOfPointsSeekBar.max = 1000000
        amountOfPointsSeekBar.progress = 400000
        //amountOfPointsSeekBar.progress = 10000
        maxPointsAmount = amountOfPointsSeekBar.progress
        amountOfPointsSeekBar.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(seekBar: SeekBar, progress: Int, fromUser: Boolean) {
                maxPointsAmount = progress
                scanPointAccumulator.maxPoints = progress
                findViewById<TextView>(R.id.amountOfPointsValue).text = (progress).toString()
            }
            override fun onStartTrackingTouch(seekBar: SeekBar) {}
            override fun onStopTrackingTouch(seekBar: SeekBar) {}
        })



        val backButton = findViewById<ImageButton>(R.id.back_button)
        backButton.setOnClickListener {  finish() }

        /**********************************
         * ********************************
         * ********************************
         *Button test
         */


        loadScanQualitySettings()
        updateModeButtonText()
        togglePlanesFilteringButton.text = if (planesFiltringEnable) "Planes On" else "Planes Off"

        //------------------
        val externalFile = File(getExternalFilesDir(null), "cords.txt")
        externalFile.writeText(" ")
        //------------------
    }

    private fun loadScanQualitySettings() {
        val prefs = getSharedPreferences("ScanSettings", android.content.Context.MODE_PRIVATE)
        val points = when (prefs.getString("quality", "medium")) {
            "low" -> 100_000
            "high" -> 1_000_000
            else -> 400_000
        }
        amountOfPointsSeekBar.progress = points
        maxPointsAmount = points
        scanPointAccumulator.maxPoints = points
    }

    private fun clearAccumulatedScan() {
        pointsCounterM = 0
        scanPointAccumulator.clear()
        renderPointBuffer = FloatBuffer.allocate(0)
        currentPoints = null
        savedPoints.clear()
        pointsAmountTextView.text = "P : 0"
    }

    private fun startScan() {
        currentMode = Mode.RAW_DEPTH
        updateModeButtonText()
        needToBeProcessed = true
        startScanButton.text = "Stop"
        settingsPanel.visibility = View.GONE
        messageSnackbarHelper.showMessage(this, "Scanning...")
    }

    private fun stopScan() {
        needToBeProcessed = false
        startScanButton.text = "Scan"
    }

    private fun toggleSettingsPanel() {
        settingsPanel.visibility = if (settingsPanel.visibility == View.VISIBLE) {
            View.GONE
        } else {
            View.VISIBLE
        }
    }

    /**
     * Saving points and converting them into a 3D model.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */

    /**
     * Here, data is read from the file point.txt and converted into a 3D model.
     * The file is located in app/assets/.
     * return: Array<FloatArray> - With prepared points for processing.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun loadPointsFromAssets(context: Context, fileName: String): Array<FloatArray> {
        val pointsList = mutableListOf<FloatArray>()

        context.assets.open(fileName).bufferedReader().use { reader ->
            reader.forEachLine { line ->
                if (line.startsWith("v ")) {
                    val parts = line.split(" ")
                    val x = parts[1].toFloat()
                    val y = parts[2].toFloat()
                    val z = parts[3].toFloat()
                    pointsList.add(floatArrayOf(x, y, z))
                }
            }
        }

        return pointsList.toTypedArray()
    }


    /**
     * Creating a link to the Downloads folder on an Android device
     * and granting permission to save.
     * return: Uri - A link to the location where the model is saved.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun createModelUri(fileName: String): Uri? {
        val prefs = getSharedPreferences("SaveSettings", android.content.Context.MODE_PRIVATE)
        val relativePath = prefs.getString("modelRelativePath", Environment.DIRECTORY_DOWNLOADS)
            ?: Environment.DIRECTORY_DOWNLOADS

        val resolver = application.contentResolver
        val values = ContentValues().apply {
            put(MediaStore.MediaColumns.DISPLAY_NAME, fileName)
            put(MediaStore.MediaColumns.MIME_TYPE, "application/octet-stream")
            put(MediaStore.MediaColumns.RELATIVE_PATH, relativePath)
        }

        val collection = if (relativePath.startsWith(Environment.DIRECTORY_DOCUMENTS)) {
            MediaStore.Files.getContentUri("external")
        } else {
            MediaStore.Downloads.EXTERNAL_CONTENT_URI
        }

        return resolver.insert(collection, values)?.also {
            Log.d("JNI", "URI: $it, path: $relativePath")
        } ?: run {
            Log.e("JNI", "Failed to create URI for saving the file.")
            null
        }
    }

    /**
     * Calling a native method,
     * passing an array of points constructed from the point cloud,
     * obtaining the model, and saving it in the Downloads folder.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun saveModelToUri(fileName: String) {
        val uri = createModelUri(fileName)
        if (uri != null) {
            val parcelFileDescriptor = contentResolver.openFileDescriptor(uri, "w")
            parcelFileDescriptor?.let {
                try {
                    // Calling a native method
                    val result = model3DCreator.processPointCloudToUri(pointArrays, it.detachFd())
                    Log.d("JNI", "The 3D model has been saved as the result: $result")
                    Toast.makeText(this, "3D model saved successfully!", Toast.LENGTH_SHORT).show()
                } catch (e: Exception) {
                    Log.e("JNI", "An error occurred while processing the point cloud to URI: ${e.message}")
                    e.printStackTrace()

                    // Displaying a Toast message with error information
                    Toast.makeText(
                        this,
                        "Failed to save 3D model: ${e.message}",
                        Toast.LENGTH_LONG
                    ).show()
                }
            } ?: run {
                Log.e("JNI", "Failed to obtain a file descriptor for saving the model.")

                // Displaying a Toast message for file descriptor error
                Toast.makeText(
                    this,
                    "Failed to obtain a file descriptor for saving the model.",
                    Toast.LENGTH_LONG
                ).show()
            }
        } else {
            Log.e("JNI", "Failed to create a URI for saving the model.")

            // Displaying a Toast message for URI creation error
            Toast.makeText(
                this,
                "Failed to create a URI for saving the model.",
                Toast.LENGTH_LONG
            ).show()
        }
    }


    /**
     * Method for saving point data in PCD format.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun savePointsToPCDFile() {
        val fileName = generateFileName("point","pcd")
        val uri = createModelUriPcd(fileName)
        val points = scanPointAccumulator.toModelBuffer()
        val pointCount = points.capacity() / 3

        if (uri != null) {
            val parcelFileDescriptor = contentResolver.openFileDescriptor(uri, "w")
            parcelFileDescriptor?.let { pfd ->
                FileOutputStream(pfd.fileDescriptor).use { fos ->
                    OutputStreamWriter(fos).use { writer ->
                        writer.write("# .PCD v0.7 - Point Cloud Data file format\n")
                        writer.write("VERSION 0.7\n")
                        writer.write("FIELDS x y z\n")
                        writer.write("SIZE 4 4 4\n")
                        writer.write("TYPE F F F\n")
                        writer.write("COUNT 1 1 1\n")

                        //Could be simplified-------------------
                        //var totalPoints = 0
                        //savedPoints.forEach { points ->
                        //    totalPoints += points.capacity() / 3
                        //}
                        writer.write("WIDTH $pointCount\n")
                        writer.write("HEIGHT 1\n")
                        writer.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                        writer.write("POINTS $pointCount\n")
                        writer.write("DATA ascii\n")

                        for (i in 0 until pointCount) {
                            val x = points.get(i * 3)
                            val y = points.get(i * 3 + 1)
                            val z = points.get(i * 3 + 2)
                            writer.write("$x $y $z\n")
                        }
                    }
                }
                pfd.close()
            }
            Log.d(TAG, "Points saved to PCD file using MediaStore.")
        } else {
            Log.e(TAG, "Failed to create URI for saving PCD file.")
        }
    }

    /**
     * Creating a link to the DOCUMENTS/PCD folder on an Android device
     *      * and granting permission to save.
     *      * return: Uri - A link to the location where the PCD is saved.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun createModelUriPcd(fileName: String): Uri? {
        val values = ContentValues().apply {
            put(MediaStore.Files.FileColumns.DISPLAY_NAME, fileName)
            put(MediaStore.Files.FileColumns.MIME_TYPE, "application/octet-stream")
            put(MediaStore.Files.FileColumns.RELATIVE_PATH, Environment.DIRECTORY_DOCUMENTS + "/PCD")
        }

        return contentResolver.insert(MediaStore.Files.getContentUri("external"), values)
    }


    /**
     * Converting all current points in the point buffer from `savedPoints`
     * captured by the device's camera into a format suitable for processing
     * by the native library, `Array<FloatArray>`,
     * and calling the main method that works with the native library.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun preparePointsForModel() {
        pointArrays = scanPointAccumulator.toModelPointArrays()
        if (pointArrays.size < MIN_POINTS_FOR_MODEL) {
            Toast.makeText(
                this,
                "Not enough stable scan points yet: ${pointArrays.size}",
                Toast.LENGTH_LONG
            ).show()
            Log.w(TAG, "Skipping model generation. Stable points: ${pointArrays.size}")
            return
        }

        val fileName =  generateFileName("model","obj")
        saveModelToUri(fileName)
    }
    /**
     * Generating a random name for files.
     * You need to provide the initial name and format.
     *  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |
     * \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/
     */
    private fun generateFileName(prefix: String, extension: String): String {
        val dateFormat = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.getDefault())
        val currentDate = dateFormat.format(Date())

        val randomHash = UUID.randomUUID().toString().take(8)

        return "$prefix${currentDate}_$randomHash.$extension"
    }






    /**********************************
     * ********************************
     * ********************************
     *Saving points
     */
    private fun fixatePoints() {
        val points = currentPoints ?: return
        val filteredPoints = filterInvalidPoints(points)
        if (filteredPoints.capacity() > 0) {
            savedPoints.clear()
            savedPoints.add(filteredPoints)
            //logPoints(filteredPoints)
            //savePointsToFile(filteredPoints)
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
        val pointCount = points.capacity() / 4

        for (i in 0 until pointCount) {
            val x = points.get(i * 4)
            val y = points.get(i * 4 + 1)
            val z = points.get(i * 4 + 2)
            val confidence = points.get(i * 4 + 3)

            if (confidence > 0f && (x != 0.0f || y != 0.0f || z != 0.0f)) {
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

    private fun xyzBufferToRenderBuffer(points: FloatBuffer, pointCount: Int): FloatBuffer {
        val renderBuffer = FloatBuffer.allocate(pointCount * 4)
        for (i in 0 until pointCount) {
            renderBuffer.put(points.get(i * 3))
            renderBuffer.put(points.get(i * 3 + 1))
            renderBuffer.put(points.get(i * 3 + 2))
            renderBuffer.put(1.0f)
        }
        renderBuffer.rewind()
        return renderBuffer
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
        togglePlanesFilteringButton.text = if (planesFiltringEnable) "Planes On" else "Planes Off"
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

            cubeRenderer.createOnGlThread(this)
            sphereRenderer.createOnGlThread(this)

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
            val frame = session!!.update()
            val camera = frame.camera

            // Camera preview renders every frame, regardless of scan state
            backgroundRenderer.draw(frame)

            if (needToBeProcessed && currentMode == Mode.RAW_DEPTH) {
                val points: FloatBuffer =
                    DepthData.create(frame, camera.pose) ?: return

                if (planesFiltringEnable) {
                    DepthData.filterUsingPlanes(points, session!!.getAllTrackables(Plane::class.java))
                }

                cameraPoseCurrent = camera.pose
                if (scanPointAccumulator.pointCount < maxPointsAmount) {
                    scanPointAccumulator.add(points)
                }
                pointsCounterM = scanPointAccumulator.pointCount
                pointsAmountTextView.text = "P : $pointsCounterM"

                renderPointBuffer = scanPointAccumulator.toRenderBuffer()
                currentPoints = renderPointBuffer
                depthRenderer.update(renderPointBuffer)
                depthRenderer.draw(camera)
            }

            if (camera.trackingState == TrackingState.PAUSED) {
                messageSnackbarHelper.showMessage(
                    this, TrackingStateHelper.getTrackingFailureReasonString(camera)
                )
            }
        } catch (t: Throwable) {
            Log.e(TAG, "Exception on the OpenGL thread", t)
        }
    }

    companion object {
        private val TAG: String = RawDepthCodelabActivity::class.java.simpleName
        private const val MIN_POINTS_FOR_MODEL = 250
    }
    //When camera mode changed, change label and visualisation
    private fun toggleMode() {
        currentMode = when (currentMode) {
            Mode.CAMERA -> Mode.RAW_DEPTH
            Mode.RAW_DEPTH -> Mode.CAMERA
        }
        if (currentMode == Mode.CAMERA) {
            stopScan()
        }
        updateModeButtonText()
    }

    private fun updateModeButtonText() {
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
