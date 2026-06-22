package com.app.threedify.helpers

import java.nio.FloatBuffer
import kotlin.math.floor
import kotlin.math.max

class ScanPointAccumulator(
    var voxelSizeMeters: Float = DEFAULT_VOXEL_SIZE_METERS,
    var minConfidence: Float = DEFAULT_MIN_CONFIDENCE,
    var maxPoints: Int = DEFAULT_MAX_POINTS,
    var minObservationsForModel: Int = DEFAULT_MIN_OBSERVATIONS,
    var keepLargestClusterOnly: Boolean = true
) {
    private val voxels = LinkedHashMap<VoxelKey, VoxelPoint>()

    val pointCount: Int
        get() = synchronized(this) { voxels.size }

    @Synchronized
    fun clear() {
        voxels.clear()
    }

    @Synchronized
    fun add(points: FloatBuffer): Int {
        var accepted = 0
        points.rewind()
        while (points.remaining() >= 4 && voxels.size < maxPoints) {
            val x = points.get()
            val y = points.get()
            val z = points.get()
            val confidence = points.get()

            if (!isValidPoint(x, y, z, confidence)) {
                continue
            }

            val key = VoxelKey.fromPoint(x, y, z, voxelSizeMeters)
            val existing = voxels[key]
            if (existing == null) {
                voxels[key] = VoxelPoint(x, y, z, confidence, 1)
                accepted++
            } else {
                existing.merge(x, y, z, confidence)
            }
        }
        return accepted
    }

    @Synchronized
    fun toRenderBuffer(): FloatBuffer {
        val buffer = FloatBuffer.allocate(voxels.size * 4)
        voxels.values.forEach { point ->
            buffer.put(point.x)
            buffer.put(point.y)
            buffer.put(point.z)
            buffer.put(point.confidence)
        }
        buffer.rewind()
        return buffer
    }

    @Synchronized
    fun toModelBuffer(): FloatBuffer {
        val stablePoints = selectModelPoints()
        val buffer = FloatBuffer.allocate(stablePoints.size * 3)
        stablePoints.forEach { point ->
            buffer.put(point.x)
            buffer.put(point.y)
            buffer.put(point.z)
        }
        buffer.rewind()
        return buffer
    }

    @Synchronized
    fun toModelPointArrays(): Array<FloatArray> {
        return selectModelPoints()
            .asSequence()
            .map { floatArrayOf(it.x, it.y, it.z) }
            .toList()
            .toTypedArray()
    }

    private fun selectModelPoints(): List<VoxelPoint> {
        val stableVoxels = voxels.filterValues { it.observations >= minObservationsForModel }
        if (!keepLargestClusterOnly || stableVoxels.size < MIN_CLUSTER_FILTER_POINTS) {
            return stableVoxels.values.toList()
        }

        val remaining = stableVoxels.keys.toMutableSet()
        var largestCluster = emptySet<VoxelKey>()
        while (remaining.isNotEmpty()) {
            val seed = remaining.first()
            val cluster = LinkedHashSet<VoxelKey>()
            val queue = ArrayDeque<VoxelKey>()
            queue.add(seed)
            remaining.remove(seed)

            while (queue.isNotEmpty()) {
                val key = queue.removeFirst()
                cluster.add(key)
                key.neighbors().forEach { neighbor ->
                    if (remaining.remove(neighbor)) {
                        queue.add(neighbor)
                    }
                }
            }

            if (cluster.size > largestCluster.size) {
                largestCluster = cluster
            }
        }

        return largestCluster.mapNotNull { stableVoxels[it] }
    }

    private fun isValidPoint(x: Float, y: Float, z: Float, confidence: Float): Boolean {
        return x.isFinite() &&
            y.isFinite() &&
            z.isFinite() &&
            confidence.isFinite() &&
            confidence >= minConfidence &&
            !(x == 0f && y == 0f && z == 0f) &&
            max(max(kotlin.math.abs(x), kotlin.math.abs(y)), kotlin.math.abs(z)) <= MAX_COORDINATE_METERS
    }

    private data class VoxelKey(val x: Int, val y: Int, val z: Int) {
        fun neighbors(): Array<VoxelKey> {
            return arrayOf(
                VoxelKey(x - 1, y, z),
                VoxelKey(x + 1, y, z),
                VoxelKey(x, y - 1, z),
                VoxelKey(x, y + 1, z),
                VoxelKey(x, y, z - 1),
                VoxelKey(x, y, z + 1)
            )
        }

        companion object {
            fun fromPoint(x: Float, y: Float, z: Float, voxelSizeMeters: Float): VoxelKey {
                return VoxelKey(
                    floor(x / voxelSizeMeters).toInt(),
                    floor(y / voxelSizeMeters).toInt(),
                    floor(z / voxelSizeMeters).toInt()
                )
            }
        }
    }

    private class VoxelPoint(
        var x: Float,
        var y: Float,
        var z: Float,
        var confidence: Float,
        var observations: Int
    ) {
        fun merge(nextX: Float, nextY: Float, nextZ: Float, nextConfidence: Float) {
            val nextWeight = nextConfidence.coerceAtLeast(0.01f)
            val currentWeight = confidence.coerceAtLeast(0.01f) * observations
            val totalWeight = currentWeight + nextWeight

            x = ((x * currentWeight) + (nextX * nextWeight)) / totalWeight
            y = ((y * currentWeight) + (nextY * nextWeight)) / totalWeight
            z = ((z * currentWeight) + (nextZ * nextWeight)) / totalWeight
            confidence = ((confidence * observations) + nextConfidence) / (observations + 1)
            observations++
        }
    }

    companion object {
        private const val DEFAULT_VOXEL_SIZE_METERS = 0.01f
        private const val DEFAULT_MIN_CONFIDENCE = 0.35f
        private const val DEFAULT_MAX_POINTS = 500_000
        private const val DEFAULT_MIN_OBSERVATIONS = 2
        private const val MIN_CLUSTER_FILTER_POINTS = 500
        private const val MAX_COORDINATE_METERS = 8f
    }
}
