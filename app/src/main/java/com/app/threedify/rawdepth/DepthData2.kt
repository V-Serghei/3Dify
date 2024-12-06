package com.app.threedify.rawdepth

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.Color
import android.graphics.ImageFormat
import android.graphics.Matrix
import android.graphics.Rect
import android.graphics.YuvImage
import android.media.Image
import android.util.Log
import com.google.ar.core.Anchor
import com.google.ar.core.Frame
import com.google.ar.core.exceptions.NotYetAvailableException
import java.io.ByteArrayOutputStream
import java.nio.FloatBuffer
import kotlin.math.round

object DepthData2 {

    private val positions3D = mutableListOf<FloatArray>()
    private val colorsRGB = mutableListOf<IntArray>()

    var confidenceThreshold: Float = 0.5f


    fun createPointCloud(frame: Frame): FloatBuffer {

        //Called on each iteration

        val pc = frame.acquirePointCloud()
        var rPoints : FloatBuffer = FloatBuffer.allocate(pc.points.limit())


        try {
            val points = pc.points
            Log.i("TAG", "points limit" + points.limit())

            val img = frame.acquireCameraImage()
            //val bmp: Bitmap = imageToBitmap(img)
            img.close()

            var i = 0
            while (i < points.limit()) {
                var x = points[i+0]
                var y = points[i+1]
                var z = points[i+2]
                var c = points[i+3]

                if (points[i + 3] < confidenceThreshold ||
                    points[i] >= 2.0f || points[i] <= -2.0f ||
                    points[i + 1] >= 2.0f || points[i + 1] <= -2.0f ||
                    points[i + 2] >= 2.0f || points[i + 2] <= -2.0f) {
                    i += 4
                    continue
                }
                //Log.i("TAG", "before $x - $y - $z - $c")


                //val w = floatArrayOf(points[i], points[i + 1], points[i + 2])

                //val color: IntArray = getScreenPixel(w, bmp)

                //if (color == null || color.size != 3) {
                //    i += 4
                //    continue
                //}

                //This two arrays store all the data we need
                //positions3D.add(floatArrayOf(points[i], points[i + 1], points[i + 2]))
                val cx = round(points[i] * 100) / 100
                val cy = round(points[i + 1] * 100) / 100
                val cz = round(points[i + 2] * 100) / 100
                val cc = round(points[i + 3] * 100) / 100

                rPoints.put(round(points[i] * 100) / 100)
                rPoints.put(round(points[i + 1] * 100) / 100)
                rPoints.put(round(points[i + 2] * 100) / 100)
                rPoints.put(round(points[i + 3] * 100) / 100)

                //Log.i("TAG", "after $cx - $cy - $cz - $cc")
                //colorsRGB.add(arrayOf<Int>(color[0], color[1], color[2]))
                i += 4
            }

            //ResourceExhaustedException - Acquire failed because there are too many objects already acquired.
            // For example, the developer may acquire up to N point clouds.
            pc.release()
        } catch (e: NotYetAvailableException) {
            Log.e("TAG", e.message!!)
        }
        //Log.i("TAG", "eeeeeeeeeeeeeeeeeeeeeeee")
        rPoints.rewind()
        return rPoints
    }

    private fun imageToBitmap(image: Image): Bitmap {
        val width = image.width
        val height = image.height

        val nv21: ByteArray
        val yBuffer = image.planes[0].buffer
        val uBuffer = image.planes[1].buffer
        val vBuffer = image.planes[2].buffer

        val ySize = yBuffer.remaining()
        val uSize = uBuffer.remaining()
        val vSize = vBuffer.remaining()

        nv21 = ByteArray(ySize + uSize + vSize)

        //U and V are swapped
        yBuffer[nv21, 0, ySize]
        vBuffer[nv21, ySize, vSize]
        uBuffer[nv21, ySize + vSize, uSize]

        val yuvImage = YuvImage(nv21, ImageFormat.NV21, width, height, null)
        val os = ByteArrayOutputStream()
        yuvImage.compressToJpeg(Rect(0, 0, width, height), 100, os)
        val jpegByteArray = os.toByteArray()
        val bitmap = BitmapFactory.decodeByteArray(jpegByteArray, 0, jpegByteArray.size)

        val matrix = Matrix()
        matrix.setRotate(90f)

        return Bitmap.createBitmap(bitmap, 0, 0, bitmap.width, bitmap.height, matrix, true)
    }

    //@Throws(NotYetAvailableException::class)
    //fun getScreenPixel(worldPos: FloatArray?, bmp: Bitmap): IntArray? {
    //    //ViewMatrix * ProjectionMatrix * Anchor Matrix
    //    //Clip to Screen Space
//
    //    val pos2D: DoubleArray = worldToScreenTranslator.worldToScreen(
    //        bmp.width,
    //        bmp.height,
    //        fragment.getArSceneView().getArFrame().getCamera(),
    //        worldPos
    //    )
//
    //    //Check if inside the screen
    //    if (pos2D[0] < 0 || pos2D[0] > bmp.width || pos2D[1] < 0 || pos2D[1] > bmp.height) {
    //        return null
    //    }
//
    //    val pixel = bmp.getPixel(pos2D[0].toInt(), pos2D[1].toInt())
    //    val r = Color.red(pixel)
    //    val g = Color.green(pixel)
    //    val b = Color.blue(pixel)
//
    //    return intArrayOf(r, g, b)
    //}

}