package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import java.lang.Math.abs

/**
 * State of an object in a frame (by default world origin frame).
 * @property r position in corresponding frame
 * @property v velocity in corresponding frame
 * @property tau local proper time object
 */
data class State(val r: Vector4, val v: Vector3, val tau: Double) {

    /**
     * @return momentarily co-moving reference frame
     */
    fun toMCRF() = Frame(r, v)

    /**
     * Transform from one frame into another.
     * @param from Frame in which the state applied (must be provided by context)
     * @param to target Frame
     */
    fun transform(from: Frame, to: Frame): State {
        //if(from == to) return this
        // TODO currently we always go over the origin space, this double transform can be combined
        val s = if (from.isOrigin()) this else
            State(
                lorentzTransformInv(from.v, this.r) + from.r,
                observedAddedVelocity(from.v, this.v),
                tau
            )
        if (to.isOrigin()) return s
        return State(
            lorentzTransform(to.v, s.r - to.r),
            transformedAddedVelocity(to.v, s.v),
            tau
        )
    }

    fun exactTau(tauAction: Double):State {
        assert(abs(tauAction - tau) < eps)
        return copy(tau = tauAction)
    }
}