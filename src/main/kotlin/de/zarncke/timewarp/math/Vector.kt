package de.zarncke.timewarp.math

import kotlin.math.*


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

    fun unit() = this * (1 / this.abs())
}

data class Vector4(val t: Double, val x: Double, val y: Double, val z: Double) : Comparable<Vector4> {
    fun to3() = Vector3(x, y, z)

    // corresponds to translation - one vector should be a difference vector
    operator fun plus(v2: Vector4) =
        Vector4(t + v2.t, x + v2.x, y + v2.y, z + v2.z)

    // should only be apllied to two place vedtors returning a difference vector
    operator fun minus(v2: Vector4) =
        Vector4(t - v2.t, x - v2.x, y - v2.y, z - v2.z)


    override fun compareTo(other: Vector4) = vector4comparator.compare(this, other)

    constructor(t: Double, v3: Vector3) : this(t, v3.x, v3.y, v3.z)
}

val vector4comparator = compareBy<Vector4>(Vector4::t, Vector4::x, Vector4::y, Vector4::z)

/**
 * https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
 */
fun eulerAxisToRotMatrix(eulerAxis: Vector3, phi: Double) = eulerAxis.unit().run {
    val sinphi = sin(phi)
    val cosphi = cos(phi)
    val omcosPhi = 1 - cosphi
    Matrix33(
        omcosPhi * x * x + cosphi, omcosPhi * x * y - z * sinphi, omcosPhi * x * z + y * sinphi,  //
        omcosPhi * y * x + z * sinphi, omcosPhi * y * y + cosphi, omcosPhi * y * z - x * sinphi, //
        omcosPhi * z * x - y * sinphi, omcosPhi * z * y + x * sinphi, omcosPhi * z * z + cosphi
    )
}

operator fun Vector3.times(matrix: Matrix33): Vector3 = this.run {
    matrix.run {
        Vector3(
            x * a11 + y * a21 + z * a31,
            x * a12 + y * a22 + z * a32,
            x * a13 + y * a23 + z * a33
        )
    }
}

fun Int.deg2Rad() = this * PI / 180
fun Double.rad2Deg() = this * 180 / PI

/**
 * Rotates a vector around the given vector in mathematical positive direction (right hand rule).
 */
fun Vector3.rotate(around: Vector3, angleRad: Double) = this.times(eulerAxisToRotMatrix(around, angleRad))

// PERFORMANCE: these could be optimized:
fun Vector3.rotateX(angleRad: Double) = this.times(eulerAxisToRotMatrix(EX, angleRad))

fun Vector3.rotateY(angleRad: Double) = this.times(eulerAxisToRotMatrix(EY, angleRad))
fun Vector3.rotateZ(angleRad: Double) = this.times(eulerAxisToRotMatrix(EZ, angleRad))


data class Matrix33(
    val a11: Double, val a12: Double, val a13: Double,
    val a21: Double, val a22: Double, val a23: Double,
    val a31: Double, val a32: Double, val a33: Double
) {
    fun toV3() = Triple(Vector3(a11, a12, a13), Vector3(a21, a22, a23), Vector3(a31, a32, a33))
    fun toArray() = arrayOf(arrayOf(a11, a12, a13), arrayOf(a21, a22, a23), arrayOf(a31, a32, a33))
    /**
     * {\displaystyle {\begin{aligned}\theta &=\arccos {\frac {A_{11}+A_{22}+A_{33}-1}{2}}\\e_{1}&={\frac {A_{32}-A_{23}}{2\sin \theta }}\\e_{2}&={\frac {A_{13}-A_{31}}{2\sin \theta }}\\e_{3}&={\frac {A_{21}-A_{12}}{2\sin \theta }}\end{aligned}}}
     * https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions
     */
    fun toEulerAxisAndAngle() = run {
        val phi = acos((a11 + a22 + a33) / 2)
        val tsp = 2 * sin(phi)
        Vector3((a32 - a23) / tsp, (a13 - a32) / tsp, (a21 - a12) / tsp) to phi
    }
}
