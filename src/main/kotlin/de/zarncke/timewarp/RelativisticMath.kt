package de.zarncke.timewarp

import java.lang.IllegalArgumentException
import kotlin.math.*


/**
 * The gamma function calculates the relativistic Lorentz factor for a velocity.
 * @param v velocity
 * @return lorentzFactor
 */
fun gamma(v: Double) = 1 / sqrt(1 - v * v)

/**
 * ...u as velocity of a body within a Lorentz frame S, and v as velocity of a second frame S′, as measured in S,
 * and u′ as the transformed velocity of the body within the second frame.
 * https://en.wikipedia.org/wiki/Velocity-addition_formula
 * http://math.ucr.edu/home/baez/physics/Relativity/SR/velocity.html
 * @param v as velocity of a second frame S′, as measured in S
 * @param u' as the velocity of the body within the second frame.
 * @return u as the \[observed] velocity of a body within a Lorentz frame S
 */
fun observedAddedVelocity(v: Vector3, uPrime: Vector3): Vector3 {
    // we do not use the cross product form because I'm not sure it's not an approximation (and more ops)
    val vAbs = v.abs()
    if (vAbs == 1.0) throw IllegalArgumentException("spaces cannot move with lightspeed")
    val a = uPrime.dot(v)
    val gamma_v = gamma(vAbs)
    return (uPrime * (1.0 / gamma_v) + v + v * a * (gamma_v / (1 + gamma_v))) * (1.0 / (1.0 + a))
}

/**
 * ...u as velocity of a body within a Lorentz frame S, and v as velocity of a second frame S′, as measured in S,
 * and u′ as the transformed velocity of the body within the second frame.
 * https://en.wikipedia.org/wiki/Velocity-addition_formula
 * http://math.ucr.edu/home/baez/physics/Relativity/SR/velocity.html
 * @param u as velocity of a body within a Lorentz frame S (aka Frame)
 * @param v as velocity of a second frame S′, as measured in S
 * @return u' as the transformed velocity of the body within the second frame.
 */
fun transformedAddedVelocity(v: Vector3, u: Vector3): Vector3 {
    // we do not use the cross product form because I'm not sure it's not an approximation (and more ops)
    val vAbs = v.abs()
    if (vAbs == 1.0) throw IllegalArgumentException("frames cannot move with lightspeed")
    val a = u.dot(v)
    val gamma_v = gamma(v.abs())
    return (u * (1.0 / gamma_v) - v + v * a * (gamma_v / (1 + gamma_v))) * (1.0 / (1.0 - a))
}

/**
 * Given a 4-vector in a frame determine 4-vector within a frame moving relative to the first frame.
 * https://en.wikipedia.org/wiki/Lorentz_transformation#Vector_transformations
 * @param v as velocity of a second frame S′, as measured in S
 * @param r as 4-vector of an event within a Lorentz frame S (aka Frame)
 * @return r' as the transformed 4-vector of the event within the second frame S'.
 */
fun lorentzTransform(v: Vector3, r: Vector4): Vector4 {
    val vAbs = v.abs()
    if (vAbs == 0.0) return r
    val n = v * (1 / vAbs)
    val r3 = r.to3()
    val gamma = gamma(vAbs)
    return Vector4(
        gamma * (r.t - v.dot(r3)),
        r3 + n * ((gamma - 1) * r3.dot(n)) - n * (gamma * r.t * vAbs)
    )
}

/**
 * Given a 4-vector in a frame moving relative to another one determine 4-vector within the second one.
 * https://en.wikipedia.org/wiki/Lorentz_transformation#Vector_transformations
 * @param v as velocity of a second frame S′, as measured in S
 * @param rPrime as 4-vector of an event within a Lorentz frame S' (aka Frame)
 * @return r as the transformed 4-vector of the event within S.
 */
fun lorentzTransformInv(v: Vector3, rPrime: Vector4): Vector4 {
    // same as lorentzTransform except for substituting -n for n
    val vAbs = v.abs()
    if (vAbs == 0.0) return rPrime
    val n = v * (1 / vAbs)
    val r3Prime = rPrime.to3()
    val gamma = gamma(vAbs)
    return Vector4(
        gamma * (rPrime.t + v.dot(r3Prime)),
        r3Prime + n * ((gamma - 1) * r3Prime.dot(n)) + v * (gamma * rPrime.t)
    )
}

/**
 * Determine location of accelerated motion for proper time.
 * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)
 * http://math.ucr.edu/home/baez/physics/Relativity/SR/Rocket/rocket.html
 * @param a0 proper acceleration in (momentarily co-moving) reference frame
 * @param tau proper time duration of the acceleration of the accelerated object
 * @return state (4-vector and velocity of the resulting position within the frame)
 */
fun relativisticAcceleration(a0: Vector3, tau: Double): State {
    val aAbs = a0.abs()
    if (aAbs == 0.0) return State(V3_0.to4(tau), V3_0, tau)
    val n = a0 * (1 / aAbs)
    return State(
        Vector4(sinh(aAbs * tau) / aAbs, n * ((cosh(aAbs * tau) - 1) / aAbs)),
        n * tanh(aAbs * tau), tau
    )
}

/**
 * Determine location of accelerated motion for coordinate time.
 * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)
 * @param a0 proper acceleration in (momentarily co-moving) reference frame
 * @param tau proper time duration of the acceleration of the accelerated object
 * @return state (4-vector and velocity of the resulting position within the frame, and proper time tau at that time)
 */
fun relativisticCoordAcceleration(a0: Vector3, t: Double): State {
    val aAbs = a0.abs()
    if (aAbs == 0.0) return State(V3_0.to4(t), V3_0, t)
    val v = aAbs * t
    val tau = ln(sqrt(1.0 + v.pow(2)) + v) / aAbs
    val n = a0 * (1 / aAbs)
    return State(Vector4(t, n * ((sqrt(1 + v.pow(2)) - 1) / aAbs)), n * tanh(aAbs * tau), tau)
}

