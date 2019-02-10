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
    fun transform(from: Frame, to: Frame) = State(this.r.transform(from, to), this.v.transformVelocity(from, to), tau)

    /**
     * Sets tau to the "correct" value (which must be within [eps]).
     * Intended to allow for exact comparisons when the correct tau is known from analytics considerations.
     */
    fun exactTau(tauCorrected: Double): State {
        assert(abs(tauCorrected - tau) < eps,
            { "fail" })
        return copy(tau = tauCorrected)
    }
}