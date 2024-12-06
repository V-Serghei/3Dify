package com.app.arcore.common.rendering

import android.content.Context
import android.opengl.GLES20
import android.opengl.Matrix
import com.google.ar.core.Camera
import java.io.IOException
import java.nio.FloatBuffer
import java.nio.ShortBuffer

class CubeRenderer {

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

    fun generateCube(size: Float): Pair<FloatArray, ShortArray> {
        val halfSize = size / 2

        // Вершины куба (x, y, z)
        val vertices = floatArrayOf(
            -halfSize, -halfSize, -halfSize,  // 0
            halfSize, -halfSize, -halfSize,   // 1
            halfSize,  halfSize, -halfSize,   // 2
            -halfSize,  halfSize, -halfSize,  // 3
            -halfSize, -halfSize,  halfSize,  // 4
            halfSize, -halfSize,  halfSize,   // 5
            halfSize,  halfSize,  halfSize,   // 6
            -halfSize,  halfSize,  halfSize   // 7
        )

        // Индексы для отрисовки куба
        val indices = shortArrayOf(
            0, 1, 2, 0, 2, 3, // Лицевая грань
            4, 5, 6, 4, 6, 7, // Задняя грань
            0, 1, 5, 0, 5, 4, // Нижняя грань
            3, 2, 6, 3, 6, 7, // Верхняя грань
            0, 3, 7, 0, 7, 4, // Левая грань
            1, 2, 6, 1, 6, 5  // Правая грань
        )

        return Pair(vertices, indices)
    }

    /** Renders the cube. */
    fun draw(camera: Camera) {
        GLES20.glUseProgram(programName)

        // Получаем проекционную и видовую матрицы
        val projectionMatrix = FloatArray(16)
        val viewMatrix = FloatArray(16)
        val viewProjection = FloatArray(16)

        camera.getProjectionMatrix(projectionMatrix, 0, 0.1f, 100.0f)
        camera.getViewMatrix(viewMatrix, 0)

        Matrix.multiplyMM(viewProjection, 0, projectionMatrix, 0, viewMatrix, 0)

        GLES20.glUniformMatrix4fv(modelViewProjectionUniform, 1, false, viewProjection, 0)

        // Генерация куба
        val (vertices, indices) = generateCube(0.01f) // Размер куба

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

        // Отрисовка куба
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, ebo[0])
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, indices.size, GLES20.GL_UNSIGNED_SHORT, 0)

        // Отключение VBO и EBO после отрисовки (опционально)
        GLES20.glBindBuffer(GLES20.GL_ARRAY_BUFFER, 0)
        GLES20.glBindBuffer(GLES20.GL_ELEMENT_ARRAY_BUFFER, 0)
    }

    companion object {
        private val TAG: String = CubeRenderer::class.java.simpleName
        // Shader names.
        private const val VERTEX_SHADER_NAME = "shaders/cube.vert"
        private const val FRAGMENT_SHADER_NAME = "shaders/cube.frag"
    }
}

