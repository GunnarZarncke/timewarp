package de.zarncke.timewarp.math

import kotlin.math.sqrt


val V3_0 = Vector3(0.0, 0.0, 0.0)
val V4_0 = Vector4(0.0, 0.0, 0.0, 0.0)

val EX = Vector3(1.0, 0.0, 0.0)
val EY = Vector3(0.0, 1.0, 0.0)
val EZ = Vector3(0.0, 0.0, 1.0)

data class Vector3(val x: Double, val y: Double, val z: Double) {
    fun to4(t: Double) = Vector4(t, x, y, z)
    operator fun times(d: Double) = Vector3(x * d, y * d, z * d)
    operator fun plus(v2: Vector3) =
        Vector3(x + v2.x, y + v2.y, z + v2.z)
    operator fun minus(v2: Vector3) =
        Vector3(x - v2.x, y - v2.y, z - v2.z)
    operator fun unaryMinus() = Vector3(-x, -y, -z)
    fun abs() = sqrt(x * x + y * y + z * z)
    fun dot(d: Vector3) = x * d.x + y * d.y + z * d.z
    fun cross(d: Vector3) =
        Vector3(y * d.z - z * d.y, z * d.x - x * d.z, x * d.y - y * d.x)
    fun norm() = this*(1/this.abs())
}

data class Vector4(val t: Double, val x: Double, val y: Double, val z: Double) {
    fun to3() = Vector3(x, y, z)
    operator fun plus(v2: Vector4) =
        Vector4(t + v2.t, x + v2.x, y + v2.y, z + v2.z)
    operator fun minus(v2: Vector4) =
        Vector4(t - v2.t, x - v2.x, y - v2.y, z - v2.z)

    constructor(t: Double, v3: Vector3) : this(t, v3.x, v3.y, v3.z)
}

data class Range(val from: Double, val to: Double) : Comparable<Range> {
    override fun compareTo(other: Range) = comparator.compare(this, other)

    companion object {
        val comparator = compareBy(Range::from, Range::to)
    }
}
