package com.app.arcore.common.rendering

import android.content.Context
import android.opengl.GLES20
import android.opengl.Matrix
import com.google.ar.core.Camera
import java.io.IOException
import java.nio.FloatBuffer
import java.nio.ShortBuffer
import kotlin.math.cos
import kotlin.math.sin

class SphereRenderer {

    private var programName = 0
    private var positionAttribute = 0
    private var modelViewProjectionUniform = 0

    @Throws(IOException::class)
    fun createOnGlThread(context: Context?) {
        // Загружаем шейдеры
        val vertexShader: Int = ShaderUtil.loadGLShader(TAG, context!!, GLES20.GL_VERTEX_SHADER, VERTEX_SHADER_NAME)
        val fragmentShader: Int = ShaderUtil.loadGLShader(TAG, context, GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_NAME)

        programName = GLES20.glCreateProgram()
        GLES20.glAttachShader(programName, vertexShader)
        GLES20.glAttachShader(programName, fragmentShader)
        GLES20.glLinkProgram(programName)
        GLES20.glUseProgram(programName)

        positionAttribute = GLES20.glGetAttribLocation(programName, "a_Position")
        modelViewProjectionUniform = GLES20.glGetUniformLocation(programName, "u_ModelViewProjection")
    }

    fun generateSphere(radius: Float, rings: Int, sectors: Int): Pair<FloatArray, ShortArray> {
        val vertices = mutableListOf<Float>()
        val indices = mutableListOf<Short>()

        // Генерация вершин
        for (i in 0..rings) {
            val theta = i * Math.PI / rings
            val sinTheta = sin(theta).toFloat()
            val cosTheta = cos(theta).toFloat()

            for (j in 0..sectors) {
                val phi = j * 2.0 * Math.PI / sectors
                val sinPhi = sin(phi).toFloat()
                val cosPhi = cos(phi).toFloat()

                val x = cosPhi * sinTheta
                val y = cosTheta
                val z = sinPhi * sinTheta

                vertices.add(radius * x)  // Координата x
                vertices.add(radius * y)  // Координата y
                vertices.add(radius * z)  // Координата z
            }
        }

        // Генерация индексов для соединения треугольников
        for (i in 0 until rings) {
            for (j in 0 until sectors) {
                val first = (i * (sectors + 1) + j).toShort()
                val second = ((i + 1) * (sectors + 1) + j).toShort()

                indices.add(first)
                indices.add(second)
                indices.add((first + 1).toShort())

                indices.add(second)
                indices.add((second + 1).toShort())
                indices.add((first + 1).toShort())
            }
        }

        return Pair(vertices.toFloatArray(), indices.toShortArray())
    }

    fun draw(camera: Camera, buffer: FloatBuffer) {
        for (i in 0 until buffer.capacity() / 4) {
            val x = buffer.get(i * 4)
            val y = buffer.get(i * 4 + 1)
            val z = buffer.get(i * 4 + 2)

            drawOneSphere(camera, x, y, z)
        }
    }

    /** Renders the sphere. */
    private fun drawOneSphere(camera: Camera, centerX: Float, centerY: Float, centerZ: Float) {
        GLES20.glUseProgram(programName)

        // Получаем проекционную и видовую матрицы
        val projectionMatrix = FloatArray(16)
        val viewMatrix = FloatArray(16)
        val modelMatrix = FloatArray(16)
        val viewProjection = FloatArray(16)
        val modelViewProjection = FloatArray(16)

        camera.getProjectionMatrix(projectionMatrix, 0, 0.1f, 100.0f)
        camera.getViewMatrix(viewMatrix, 0)

        // Создаем матрицу модели и применяем трансляцию к центру сферы
        Matrix.setIdentityM(modelMatrix, 0)
        Matrix.translateM(modelMatrix, 0, centerX, centerY, centerZ)

        // Умножаем матрицы: сначала модель на вид, затем на проекцию
        Matrix.multiplyMM(viewProjection, 0, viewMatrix, 0, modelMatrix, 0)
        Matrix.multiplyMM(modelViewProjection, 0, projectionMatrix, 0, viewProjection, 0)

        // Передаем матрицу Model-View-Projection в шейдер
        GLES20.glUniformMatrix4fv(modelViewProjectionUniform, 1, false, modelViewProjection, 0)

        // Генерация сферы
        val (vertices, indices) = generateSphere(0.01f, 15, 15) // radius ~ 1cm

        // Создание и настройка VBO (Vertex Buffer Object)
        val vbo = IntArray(1)
        GLES20.glGenBuffers(1, vbo, 0)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, vbo[0])
        GLES20.glBufferData(GLES20.GL_ARRAY_BUFFER, vertices.size * 4, FloatBuffer.wrap(vertices), GLES20.GL_STATIC_DRAW)

        // Создание и настройка EBO (Element Buffer Object)
        val ebo = IntArray(1)
        GLES20.glGenBuffers(1, ebo, 0)
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ebo[0])
        GLES20.glBufferData(GLES20.GL_ELEMENT_ARRAY_BUFFER, indices.size * 2, ShortBuffer.wrap(indices), GLES20.GL_STATIC_DRAW)

        // Установите указатели на атрибуты вершин
        GLES20.glVertexAttribPointer(positionAttribute, 3, GLES20.GL_FLOAT, false, 3 * 4, 0)
        GLES20.glEnableVertexAttribArray(positionAttribute)

        // Отрисовка сферы
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ebo[0])
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, indices.size, GLES20.GL_UNSIGNED_SHORT, 0)

        // Отключение VBO и EBO после отрисовки (опционально)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, 0)
    }


    companion object {
        private val TAG: String = SphereRenderer::class.java.simpleName
        // Shader names.
        private const val VERTEX_SHADER_NAME = "shaders/sphere.vert"
        private const val FRAGMENT_SHADER_NAME = "shaders/sphere.frag"
    }
}
