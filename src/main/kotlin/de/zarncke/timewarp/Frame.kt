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