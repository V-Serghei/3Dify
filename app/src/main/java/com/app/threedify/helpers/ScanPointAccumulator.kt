package com.app.threedify.helpers

import java.nio.FloatBuffer
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.floor
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class ScanPointAccumulator(
    var voxelSizeMeters: Float = DEFAULT_VOXEL_SIZE_METERS,
    var minConfidence: Float = DEFAULT_MIN_CONFIDENCE,
    var maxPoints: Int = DEFAULT_MAX_POINTS,
    var minObservationsForModel: Int = DEFAULT_MIN_OBSERVATIONS,
    var keepLargestClusterOnly: Boolean = true,
    var selectionMode: ScanSelectionMode = ScanSelectionMode.LARGEST_CLUSTER
) {
    private val voxels = LinkedHashMap<VoxelKey, VoxelPoint>()
    private var latestCameraObservation: CameraObservation? = null

    val pointCount: Int
        get() = synchronized(this) { voxels.size }

    @Synchronized
    fun clear() {
        voxels.clear()
    }

    @Synchronized
    fun updateCameraObservation(observation: CameraObservation) {
        latestCameraObservation = observation.normalized()
    }

    @Synchronized
    fun removePointsInside(bounds: ScanBounds): Int {
        val before = voxels.size
        val iterator = voxels.entries.iterator()
        while (iterator.hasNext()) {
            if (bounds.contains(iterator.next().value)) {
                iterator.remove()
            }
        }
        return before - voxels.size
    }

    @Synchronized
    fun keepOnlyPointsInside(bounds: ScanBounds): Int {
        val before = voxels.size
        val iterator = voxels.entries.iterator()
        while (iterator.hasNext()) {
            if (!bounds.contains(iterator.next().value)) {
                iterator.remove()
            }
        }
        return before - voxels.size
    }

    @Synchronized
    fun removePointsMatching(predicate: (ScanPoint) -> Boolean): Int {
        val before = voxels.size
        val iterator = voxels.entries.iterator()
        while (iterator.hasNext()) {
            if (predicate(iterator.next().value)) {
                iterator.remove()
            }
        }
        return before - voxels.size
    }

    @Synchronized
    fun keepOnlyPointsMatching(predicate: (ScanPoint) -> Boolean): Int {
        val before = voxels.size
        val iterator = voxels.entries.iterator()
        while (iterator.hasNext()) {
            if (!predicate(iterator.next().value)) {
                iterator.remove()
            }
        }
        return before - voxels.size
    }

    @Synchronized
    fun add(
        points: FloatBuffer,
        cameraObservation: CameraObservation? = latestCameraObservation
    ): Int {
        var accepted = 0
        val observation = cameraObservation?.normalized()
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
                voxels[key] = VoxelPoint(x, y, z, confidence, 1).also {
                    it.observeFrom(observation)
                }
                accepted++
            } else {
                existing.merge(x, y, z, confidence, observation)
            }
        }
        return accepted
    }

    @Synchronized
    fun toRenderBuffer(): FloatBuffer {
        return toRenderBuffer(voxels.values)
    }

    @Synchronized
    fun toModelRenderBuffer(): FloatBuffer {
        return toRenderBuffer(selectModelPoints())
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

    @Synchronized
    fun keepOnlyModelPoints(): Int {
        val selectedKeys = selectModelKeys()
        if (selectedKeys.isEmpty()) {
            return 0
        }

        val before = voxels.size
        val iterator = voxels.entries.iterator()
        while (iterator.hasNext()) {
            if (iterator.next().key !in selectedKeys) {
                iterator.remove()
            }
        }
        return before - voxels.size
    }

    @Synchronized
    fun keepOnlyLikelyObjectPoints(): Int {
        selectionMode = ScanSelectionMode.OBJECT_AWARE
        return keepOnlyModelPoints()
    }

    @Synchronized
    fun analyze(): ScanQualityReport {
        val totalPoints = voxels.size
        if (totalPoints == 0) {
            return ScanQualityReport(
                totalPoints = 0,
                stablePoints = 0,
                modelPoints = 0,
                largestClusterPoints = 0,
                averageConfidence = 0f,
                averageObservations = 0f,
                averageJitterMeters = 0f,
                coverageScore = 0f,
                angleCoverageBins = 0,
                angleCoveragePercent = 0,
                readinessScore = 0f,
                selectionMode = selectionMode,
                status = ScanReadiness.EMPTY,
                hint = "Aim at the object and press Scan"
            )
        }

        val stableVoxels = stableVoxels()
        val modelPoints = selectModelPoints()
        val averageConfidence = voxels.values.map { it.confidence }.average().toFloat()
        val averageObservations = voxels.values.map { it.observations }.average().toFloat()
        val averageJitter = stableVoxels.values.map { it.averageJitterMeters }.averageOrZero()
        val coverageScore = estimateDensityCoverage(modelPoints)
        val angleCoverageBins = countAngleBins(modelPoints)
        val angleCoveragePercent = (angleCoverageBins * 100 / ANGLE_BINS).coerceIn(0, 100)
        val readinessScore = computeReadinessScore(
            stablePoints = stableVoxels.size,
            modelPoints = modelPoints.size,
            averageConfidence = averageConfidence,
            densityCoverageScore = coverageScore,
            angleCoveragePercent = angleCoveragePercent,
            averageJitterMeters = averageJitter
        )
        val status = when {
            modelPoints.size < MIN_USABLE_MODEL_POINTS -> ScanReadiness.NEEDS_MORE_POINTS
            angleCoverageBins < MIN_READY_ANGLE_BINS -> ScanReadiness.NEEDS_MORE_ANGLES
            coverageScore < 0.18f -> ScanReadiness.NEEDS_MORE_DENSITY
            averageConfidence < 0.45f -> ScanReadiness.NOISY_DEPTH
            averageJitter > maxTemporalJitterMeters() -> ScanReadiness.UNSTABLE_DEPTH
            readinessScore >= 0.82f -> ScanReadiness.READY
            else -> ScanReadiness.IMPROVING
        }

        return ScanQualityReport(
            totalPoints = totalPoints,
            stablePoints = stableVoxels.size,
            modelPoints = modelPoints.size,
            largestClusterPoints = modelPoints.size,
            averageConfidence = averageConfidence,
            averageObservations = averageObservations,
            averageJitterMeters = averageJitter,
            coverageScore = coverageScore,
            angleCoverageBins = angleCoverageBins,
            angleCoveragePercent = angleCoveragePercent,
            readinessScore = readinessScore,
            selectionMode = selectionMode,
            status = status,
            hint = status.hint
        )
    }

    @Synchronized
    fun debugSnapshot(maxSamplePoints: Int = 300): String {
        val report = analyze()
        val builder = StringBuilder()
        builder.appendLine("summary")
        builder.appendLine("total_points,stable_points,model_points,confidence,observations,jitter_m,coverage,angles,mode,status")
        builder.appendLine(
            "${report.totalPoints},${report.stablePoints},${report.modelPoints}," +
                "${report.averageConfidence},${report.averageObservations},${report.averageJitterMeters}," +
                "${report.coverageScore},${report.angleCoverageBins},${report.selectionMode},${report.status}"
        )
        builder.appendLine("sample_points")
        builder.appendLine("x,y,z,confidence,observations,jitter_m,angle_mask")
        voxels.values.asSequence()
            .take(maxSamplePoints)
            .forEach { point ->
                builder.appendLine(
                    "${point.x},${point.y},${point.z},${point.confidence}," +
                        "${point.observations},${point.averageJitterMeters},${point.angleMask}"
                )
            }
        return builder.toString()
    }

    private fun stableVoxels(): Map<VoxelKey, VoxelPoint> {
        return voxels.filterValues { it.isStableForModel(minObservationsForModel, maxTemporalJitterMeters()) }
    }

    private fun selectModelPoints(): List<VoxelPoint> {
        val stableVoxels = stableVoxels()
        val selectedKeys = selectModelKeys(stableVoxels)
        if (selectedKeys.isEmpty()) {
            return emptyList()
        }
        return selectedKeys.mapNotNull { stableVoxels[it] }
    }

    private fun selectModelKeys(
        stableVoxels: Map<VoxelKey, VoxelPoint> = stableVoxels()
    ): Set<VoxelKey> {
        if (!keepLargestClusterOnly || stableVoxels.size < MIN_CLUSTER_FILTER_POINTS) {
            return stableVoxels.keys
        }

        return when (selectionMode) {
            ScanSelectionMode.LARGEST_CLUSTER -> selectLargestClusterKeys(stableVoxels)
            ScanSelectionMode.OBJECT_AWARE -> selectObjectAwareKeys(stableVoxels)
        }
    }

    private fun selectLargestClusterKeys(stableVoxels: Map<VoxelKey, VoxelPoint>): Set<VoxelKey> {
        return collectClusters(stableVoxels).maxByOrNull { it.size } ?: emptySet()
    }

    private fun selectObjectAwareKeys(stableVoxels: Map<VoxelKey, VoxelPoint>): Set<VoxelKey> {
        val clusters = collectClusters(stableVoxels)
        if (clusters.isEmpty()) {
            return emptySet()
        }

        val stats = clusters.map { cluster -> buildClusterStats(cluster, stableVoxels) }
        val best = stats
            .filter { it.pointCount >= MIN_OBJECT_CLUSTER_POINTS && !it.isLikelyLargePlane }
            .maxByOrNull { it.objectScore }
            ?: stats.maxByOrNull { it.objectScore }

        return best?.keys ?: selectLargestClusterKeys(stableVoxels)
    }

    private fun collectClusters(stableVoxels: Map<VoxelKey, VoxelPoint>): List<Set<VoxelKey>> {
        val remaining = stableVoxels.keys.toMutableSet()
        val clusters = mutableListOf<Set<VoxelKey>>()
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

            clusters.add(cluster)
        }
        return clusters
    }

    private fun buildClusterStats(
        keys: Set<VoxelKey>,
        stableVoxels: Map<VoxelKey, VoxelPoint>
    ): ClusterStats {
        var minX = Float.MAX_VALUE
        var minY = Float.MAX_VALUE
        var minZ = Float.MAX_VALUE
        var maxX = -Float.MAX_VALUE
        var maxY = -Float.MAX_VALUE
        var maxZ = -Float.MAX_VALUE
        var sumX = 0f
        var sumY = 0f
        var sumZ = 0f
        var sumConfidence = 0f
        var sumObservations = 0f
        var sumJitter = 0f
        var angleMask = 0

        keys.forEach { key ->
            val point = stableVoxels[key] ?: return@forEach
            minX = min(minX, point.x)
            minY = min(minY, point.y)
            minZ = min(minZ, point.z)
            maxX = max(maxX, point.x)
            maxY = max(maxY, point.y)
            maxZ = max(maxZ, point.z)
            sumX += point.x
            sumY += point.y
            sumZ += point.z
            sumConfidence += point.confidence
            sumObservations += point.observations
            sumJitter += point.averageJitterMeters
            angleMask = angleMask or point.angleMask
        }

        val count = keys.size.coerceAtLeast(1)
        val centerX = sumX / count
        val centerY = sumY / count
        val centerZ = sumZ / count
        val spanX = (maxX - minX).coerceAtLeast(voxelSizeMeters)
        val spanY = (maxY - minY).coerceAtLeast(voxelSizeMeters)
        val spanZ = (maxZ - minZ).coerceAtLeast(voxelSizeMeters)
        val spans = listOf(spanX, spanY, spanZ).sorted()
        val volume = (spanX * spanY * spanZ).coerceAtLeast(voxelSizeMeters * voxelSizeMeters * voxelSizeMeters)
        val density = count / volume
        val latestCamera = latestCameraObservation
        val distanceToCamera = latestCamera?.distanceTo(centerX, centerY, centerZ) ?: 1f
        val frontScore = latestCamera?.frontScore(centerX, centerY, centerZ) ?: 0.65f
        val flatnessRatio = spans.first() / spans.last().coerceAtLeast(voxelSizeMeters)
        val shapeScore = ((flatnessRatio - 0.04f) / 0.26f).coerceIn(0f, 1f)
        val densityScore = (density / OBJECT_DENSITY_TARGET).coerceIn(0f, 1f)
        val countScore = (count / TARGET_MODEL_POINTS.toFloat()).coerceIn(0f, 1f)
        val distanceScore = (1f - abs(distanceToCamera - 0.85f) / 1.35f).coerceIn(0f, 1f)
        val stabilityScore = ((sumObservations / count) / 6f).coerceIn(0f, 1f)
        val jitterScore = (1f - ((sumJitter / count) / maxTemporalJitterMeters())).coerceIn(0f, 1f)
        val angleScore = (countBits(angleMask) / MIN_READY_ANGLE_BINS.toFloat()).coerceIn(0f, 1f)
        val sizeScore = when {
            spans.last() < MIN_OBJECT_SPAN_METERS -> 0.25f
            spans.last() > MAX_OBJECT_SPAN_METERS -> 0.35f
            else -> 1f
        }
        val isLikelyLargePlane = spans.last() > LARGE_PLANE_SPAN_METERS && flatnessRatio < 0.09f
        val planePenalty = if (isLikelyLargePlane) 0.18f else 1f
        val objectScore = (
            countScore * 0.23f +
                densityScore * 0.18f +
                shapeScore * 0.18f +
                frontScore * 0.14f +
                distanceScore * 0.10f +
                stabilityScore * 0.07f +
                jitterScore * 0.05f +
                angleScore * 0.05f
            ) * sizeScore * planePenalty

        return ClusterStats(keys, count, objectScore, isLikelyLargePlane)
    }

    private fun estimateDensityCoverage(points: List<VoxelPoint>): Float {
        if (points.size < MIN_USABLE_MODEL_POINTS) {
            return 0f
        }

        var minX = Float.MAX_VALUE
        var minY = Float.MAX_VALUE
        var minZ = Float.MAX_VALUE
        var maxX = -Float.MAX_VALUE
        var maxY = -Float.MAX_VALUE
        var maxZ = -Float.MAX_VALUE

        points.forEach { point ->
            minX = min(minX, point.x)
            minY = min(minY, point.y)
            minZ = min(minZ, point.z)
            maxX = max(maxX, point.x)
            maxY = max(maxY, point.y)
            maxZ = max(maxZ, point.z)
        }

        val spanX = max(1, floor((maxX - minX) / voxelSizeMeters).toInt() + 1)
        val spanY = max(1, floor((maxY - minY) / voxelSizeMeters).toInt() + 1)
        val spanZ = max(1, floor((maxZ - minZ) / voxelSizeMeters).toInt() + 1)
        val boxVolume = spanX.toLong() * spanY.toLong() * spanZ.toLong()
        if (boxVolume <= 0L) {
            return 0f
        }

        val density = points.size / boxVolume.toFloat()
        return density.coerceIn(0f, 1f)
    }

    private fun computeReadinessScore(
        stablePoints: Int,
        modelPoints: Int,
        averageConfidence: Float,
        densityCoverageScore: Float,
        angleCoveragePercent: Int,
        averageJitterMeters: Float
    ): Float {
        val stableScore = (stablePoints / TARGET_STABLE_POINTS.toFloat()).coerceIn(0f, 1f)
        val modelScore = (modelPoints / TARGET_MODEL_POINTS.toFloat()).coerceIn(0f, 1f)
        val confidenceScore = ((averageConfidence - 0.25f) / 0.5f).coerceIn(0f, 1f)
        val densityScore = (densityCoverageScore / 0.35f).coerceIn(0f, 1f)
        val angleScore = (angleCoveragePercent / 75f).coerceIn(0f, 1f)
        val temporalScore = (1f - (averageJitterMeters / maxTemporalJitterMeters())).coerceIn(0f, 1f)
        return (
            stableScore * 0.22f +
                modelScore * 0.24f +
                confidenceScore * 0.16f +
                densityScore * 0.14f +
                angleScore * 0.16f +
                temporalScore * 0.08f
            ).coerceIn(0f, 1f)
    }

    private fun countAngleBins(points: List<VoxelPoint>): Int {
        var mask = 0
        points.forEach { point -> mask = mask or point.angleMask }
        return countBits(mask)
    }

    private fun countBits(value: Int): Int {
        var bits = 0
        var next = value
        while (next != 0) {
            bits += next and 1
            next = next ushr 1
        }
        return bits
    }

    private fun toRenderBuffer(points: Collection<VoxelPoint>): FloatBuffer {
        val buffer = FloatBuffer.allocate(points.size * 4)
        points.forEach { point ->
            buffer.put(point.x)
            buffer.put(point.y)
            buffer.put(point.z)
            buffer.put(point.confidence)
        }
        buffer.rewind()
        return buffer
    }

    private fun maxTemporalJitterMeters(): Float {
        return max(MIN_TEMPORAL_JITTER_METERS, voxelSizeMeters * TEMPORAL_JITTER_VOXEL_MULTIPLIER)
    }

    private fun isValidPoint(x: Float, y: Float, z: Float, confidence: Float): Boolean {
        return x.isFinite() &&
            y.isFinite() &&
            z.isFinite() &&
            confidence.isFinite() &&
            confidence >= minConfidence &&
            !(x == 0f && y == 0f && z == 0f) &&
            max(max(abs(x), abs(y)), abs(z)) <= MAX_COORDINATE_METERS
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
        override var x: Float,
        override var y: Float,
        override var z: Float,
        var confidence: Float,
        var observations: Int
    ) : ScanPoint {
        var angleMask: Int = 0
            private set
        private var jitterSumMeters: Float = 0f

        val averageJitterMeters: Float
            get() = if (observations <= 1) 0f else jitterSumMeters / (observations - 1)

        fun merge(
            nextX: Float,
            nextY: Float,
            nextZ: Float,
            nextConfidence: Float,
            observation: CameraObservation?
        ) {
            val shift = distance(x, y, z, nextX, nextY, nextZ)
            val nextWeight = nextConfidence.coerceAtLeast(0.01f)
            val currentWeight = confidence.coerceAtLeast(0.01f) * observations
            val totalWeight = currentWeight + nextWeight

            x = ((x * currentWeight) + (nextX * nextWeight)) / totalWeight
            y = ((y * currentWeight) + (nextY * nextWeight)) / totalWeight
            z = ((z * currentWeight) + (nextZ * nextWeight)) / totalWeight
            confidence = ((confidence * observations) + nextConfidence) / (observations + 1)
            jitterSumMeters += shift
            observations++
            observeFrom(observation)
        }

        fun observeFrom(observation: CameraObservation?) {
            observation ?: return
            val angle = atan2(
                (observation.x - x).toDouble(),
                (observation.z - z).toDouble()
            )
            val normalized = ((angle + Math.PI) / (Math.PI * 2.0)).coerceIn(0.0, 0.999999)
            val bin = floor(normalized * ANGLE_BINS).toInt().coerceIn(0, ANGLE_BINS - 1)
            angleMask = angleMask or (1 shl bin)
        }

        fun isStableForModel(minObservations: Int, maxJitterMeters: Float): Boolean {
            return observations >= minObservations &&
                averageJitterMeters <= maxJitterMeters
        }

        private fun distance(ax: Float, ay: Float, az: Float, bx: Float, by: Float, bz: Float): Float {
            val dx = ax - bx
            val dy = ay - by
            val dz = az - bz
            return sqrt(dx * dx + dy * dy + dz * dz)
        }
    }

    private data class ClusterStats(
        val keys: Set<VoxelKey>,
        val pointCount: Int,
        val objectScore: Float,
        val isLikelyLargePlane: Boolean
    )

    companion object {
        private const val DEFAULT_VOXEL_SIZE_METERS = 0.01f
        private const val DEFAULT_MIN_CONFIDENCE = 0.35f
        private const val DEFAULT_MAX_POINTS = 500_000
        private const val DEFAULT_MIN_OBSERVATIONS = 2
        private const val MIN_CLUSTER_FILTER_POINTS = 500
        private const val MIN_USABLE_MODEL_POINTS = 1_000
        private const val MIN_OBJECT_CLUSTER_POINTS = 350
        private const val TARGET_STABLE_POINTS = 12_000
        private const val TARGET_MODEL_POINTS = 8_000
        private const val MAX_COORDINATE_METERS = 8f
        private const val ANGLE_BINS = 8
        private const val MIN_READY_ANGLE_BINS = 4
        private const val MIN_TEMPORAL_JITTER_METERS = 0.018f
        private const val TEMPORAL_JITTER_VOXEL_MULTIPLIER = 2.2f
        private const val OBJECT_DENSITY_TARGET = 120_000f
        private const val MIN_OBJECT_SPAN_METERS = 0.05f
        private const val MAX_OBJECT_SPAN_METERS = 2.5f
        private const val LARGE_PLANE_SPAN_METERS = 1.4f
    }
}

private fun Iterable<Float>.averageOrZero(): Float {
    var sum = 0f
    var count = 0
    forEach {
        sum += it
        count++
    }
    return if (count == 0) 0f else sum / count
}

data class CameraObservation(
    val x: Float,
    val y: Float,
    val z: Float,
    val forwardX: Float,
    val forwardY: Float,
    val forwardZ: Float
) {
    fun normalized(): CameraObservation {
        val length = sqrt(forwardX * forwardX + forwardY * forwardY + forwardZ * forwardZ)
        if (length <= 0.0001f) {
            return this
        }
        return copy(
            forwardX = forwardX / length,
            forwardY = forwardY / length,
            forwardZ = forwardZ / length
        )
    }

    fun distanceTo(pointX: Float, pointY: Float, pointZ: Float): Float {
        val dx = pointX - x
        val dy = pointY - y
        val dz = pointZ - z
        return sqrt(dx * dx + dy * dy + dz * dz)
    }

    fun frontScore(pointX: Float, pointY: Float, pointZ: Float): Float {
        val dx = pointX - x
        val dy = pointY - y
        val dz = pointZ - z
        val length = sqrt(dx * dx + dy * dy + dz * dz)
        if (length <= 0.0001f) {
            return 0f
        }
        val dot = (dx / length) * forwardX + (dy / length) * forwardY + (dz / length) * forwardZ
        return ((dot - 0.15f) / 0.85f).coerceIn(0f, 1f)
    }
}

data class ScanBounds(
    val minX: Float,
    val minY: Float,
    val minZ: Float,
    val maxX: Float,
    val maxY: Float,
    val maxZ: Float
) {
    fun contains(point: ScanPoint): Boolean {
        return point.x in minX..maxX &&
            point.y in minY..maxY &&
            point.z in minZ..maxZ
    }
}

interface ScanPoint {
    val x: Float
    val y: Float
    val z: Float
}

data class ScanQualityReport(
    val totalPoints: Int,
    val stablePoints: Int,
    val modelPoints: Int,
    val largestClusterPoints: Int,
    val averageConfidence: Float,
    val averageObservations: Float,
    val averageJitterMeters: Float,
    val coverageScore: Float,
    val angleCoverageBins: Int,
    val angleCoveragePercent: Int,
    val readinessScore: Float,
    val selectionMode: ScanSelectionMode,
    val status: ScanReadiness,
    val hint: String
) {
    val readinessPercent: Int
        get() = (readinessScore * 100).toInt().coerceIn(0, 100)
}

enum class ScanSelectionMode(val label: String) {
    LARGEST_CLUSTER("Cluster"),
    OBJECT_AWARE("Object")
}

enum class ScanReadiness(val hint: String) {
    EMPTY("Aim at the object and press Scan"),
    NEEDS_MORE_POINTS("Collect more stable points"),
    NEEDS_MORE_ANGLES("Move around the object; side/back coverage is missing"),
    NEEDS_MORE_DENSITY("Scan slower to fill gaps in the object"),
    NOISY_DEPTH("Depth is noisy; improve lighting or raise confidence"),
    UNSTABLE_DEPTH("Hold steadier; depth points are jumping"),
    IMPROVING("Keep scanning slowly"),
    READY("Ready to create model")
}
