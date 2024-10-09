#version 100
attribute vec4 a_Position; // Позиция вершины (x, y, z, w)
uniform mat4 u_ModelViewProjection; // Модельно-видовая-проекционная матрица

void main() {
    // Преобразование позиции вершины из модельного пространства в экранное
    gl_Position = u_ModelViewProjection * a_Position;
}
