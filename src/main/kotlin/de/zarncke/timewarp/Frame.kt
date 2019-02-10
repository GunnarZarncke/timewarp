package de.zarncke.timewarp

import de.zarncke.timewarp.math.*

/**
 * A frame is a coordinate system that is relative to another coordinate system (by default the origin).
 * Note: No rotation supported yet.
 * @property r relative position to other frame as measured by the other frame
 * @property v relative velocity as measured by the other frame
 */
data class Frame(val r: Vector4, val v: Vector3) {
    companion object {
        val ORIGIN = Frame(V4_0, V3_0)
    }

    fun isOrigin() = r == V4_0 && v == V3_0

    fun boost(v: Vector3) = Frame(
        lorentzTransform(v, r),
        transformedAddedVelocity(v, this.v)
    )
}

/**
 * Transform vector from one frame into another.
 * See also the more general [State.transform].
 * @param from Frame in which the vector applies (must be provided by context)
 * @param to target Frame
 */
fun Vector4.transform(from: Frame, to: Frame): Vector4{
    if(from == to) return this
    // TODO currently we always go over the origin space, this double transform can be combined
    val r = if (from.isOrigin()) this else
        lorentzTransformInv(from.v, this) + from.r
    if (to.isOrigin()) return r
    return lorentzTransform(to.v, r - to.r)
}

/**
 * Transform velocity from one frame into another.
 * See also the more general [State.transform].
 * @param from Frame in which the vector applies (must be provided by context)
 * @param to target Frame
 */
fun Vector3.transformVelocity(from: Frame, to: Frame): Vector3{
    if(from == to) return this
    // TODO currently we always go over the origin space, this double transform can be combined
    val v = if (from.isOrigin()) this else
        observedAddedVelocity(from.v, this)
    if (to.isOrigin()) return v
    return transformedAddedVelocity(to.v, v)
}
