package de.zarncke.timewarp.math

import de.zarncke.timewarp.*
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
    val sinh = sinh(aAbs * tau)
    val cosh = cosh(aAbs * tau)
    val tanh = sinh / cosh
    return State(Vector4(sinh / aAbs, n * ((cosh - 1) / aAbs)), n * tanh, tau)
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
    val tau = asinh(v) / aAbs
    val n = a0 * (1 / aAbs)
    return State(
        Vector4(
            t,
            n * ((sqrt(1 + sqr(v)) - 1) / aAbs)
        ), n * tanh(aAbs * tau), tau
    )
}

/**
 * Determine state (location, velocity, proper time) of accelerated motion for coordinate time
 * <em>in a different coordinate frame</em>.
 * <p/>
 * We start with the longitudinally accelerated (hyperbolic) motion in a reference frame
 * co-moving at the origin/start of the motion (see [relativisticAcceleration]):
 * $$r'(\tau) = 1/\alpha * (cosh \alpha \tau - 1$$
 * $$t'(\tau) = 1/\alpha * sinh \alpha \tau$$
 * with \(\alpha\) the constant acceleration in an inertial momentarily co-moving reference frame).
 * For the general case of acceleration into arbitrary coordinate directions we introduce a
 * unit vector \(\vv*n\alpha\):
 * $$\vv r'(tau) = r'(tau) * \vv*n\alpha$$
 * Now we apply an inverse Lorentz boost to express the coordinates in the origin frame
 * $$t(\tau) = \gamma (t'(\tau) + \vv r'(\tau) \cdot \vv v$$
 * $$        = \gamma (t'(\tau) + r'(\tau) \vv*n\alpha \cdot \vv v$$
 * Where \(\gamma = \gamma(v)\) is the usual Lorentz length contraction gamma and v is the relative velocity
 * of the accelerated frame relative to the origin frame. In vectorial form
 * $$\vv v = v * \vv*nv = const$$
 * Where \(\vv*nv\) is a unit vector in the direction of the relative motion.
 * </p>
 * We can express r' in terms of t'
 * $$r'(\tau) = 1/\alpha (\sqrt{(\alpha t'(\tau))^2+1} - 1)$$
 * because of the hyperbolic law
 * $$              cosh x          =          \sqrt{sinh^2 x + 1}$$
 * $$ <=>          cosh \alpha\tau =          \sqrt{sinh^2 \alpha\tau}$$
 * $$ <=> 1/\alpha cosh \alpha\tau = 1/\alpha \sqrt{sinh^2 \alpha\tau}$$
 * We now expand t' and r' in \(t(\tau)\)
 * $$t(\tau) = \gamma (t'(\tau) + r'(\tau) \vv*n\alpha \cdot v \vv nv)$$
 * $$        = \gamma (t'(\tau) + 1/\alpha (\sqrt{(\alpha t'(\tau))^2+1} - 1) \vv*n\alpha \cdot v \vv nv)$$
 * To keep this manageable we call \(\vv*n\alpha \cdot v \vv nv) = w\) and simplify:
 * $$t(\tau) = \gamma (t'(\tau) + w\sqrt{t'(\tau)^2+1/\alpha^2} - w/\alpha)$$
 * We solve this with Maxima for \(t'(\tau)\):
 * <pre>
 *  to_poly_solve(t=gamma*(tp+w*sqrt(tp^2+1/alpha^2)-w/alpha),tp);
 * </pre>
 * $$t'(\tau) = { -w/\gamma\sqrt{(\alpha t(\tau))^2 + \gamma^2 + 2\alpha t(\tau)\gamma w} + w + \alpha/\gamma t(\tau) \over \alpha(1 - w^2)}$$
 * We take the negative square root because it is the only physical solution (and the constraints for complex valued arguments don't apply).
 * Because
 * $$t'(\tau) = 1/\alpha * sinh \alpha \tau => \tau = \alpha sinh^{-1} \alpha t$$
 * we can now solve for \(\tau\):
 * $$\tau = 1/\alpha sinh^{-1}{ -w/\gamma\sqrt{(\alpha t(\tau))^2 + \gamma^2 + 2\alpha t(\tau)\gamma w} + w + \alpha/\gamma t(\tau) \over w^2 - 1}$$
 * And slightly reorder factors:
 * $$\tau = 1/\alpha sinh^{-1}{ -w\sqrt{(\alpha/\gamma t(\tau))^2 + 1 + 2\alpha/\gamma t(\tau) w} + w + \alpha/\gamma t(\tau) \over w^2 - 1}$$
 * (note how t only occurs in terms \like \(alpha/\gamma t(\tau)\))
 * A simple check shows that for \(\v = 0\) we have \(\tau = 1/\alpha sinh^{-1} \alpha t\) as expected.
 * We can also derive e.g. that for \(\vv v \bot \vv r\) we have \(\tau = 1/\alpha sinh^{-1}{\alpha t \over \gamma}\).
 * Now that we have tau we can apply [relativisticAcceleration] to derive the rest.
 * @param a0 proper acceleration in (momentarily co-moving) reference frame
 * @param t coordinate time in the observer frame at the end of the acceleration motion of the object
 * @param f frame of co-moving observer at the start of the acceleration (with velocity v relative to origin frame)
 * @return state (4-vector and velocity of the resulting position within the frame) in the origin frame
 */
fun relativisticCoordAcceleration(a0: Vector3, t: Double, frame: Frame): State {
    val dt = t - frame.r.t
    val v = frame.v
    if (v == V3_0) return relativisticCoordAcceleration(a0, dt).transform(frame, Frame.ORIGIN) // starting inertial
    if (a0 == V3_0) return lorentzTransform(v, V3_0.to4(dt)).let { State(it + frame.r, v, it.t) } // continue inertial
    val aAbs = a0.abs()
    val na = a0 * (1 / aAbs)
    val vAbs = v.abs()
    if (vAbs >= 1.0) throw throw IllegalArgumentException("frames cannot move with lightspeed")
    val gamma = gamma(vAbs)

    val w = v.dot(na)
    val atg = aAbs * dt / gamma
    val wsqrt = w * sqrt(atg * atg + 2 * atg * w + 1)
    assert(wsqrt <= w + atg)
    val tau = asinh((-wsqrt + w + atg) / (1 - w * w)) / aAbs

    return relativisticAcceleration(a0, tau).transform(frame, Frame.ORIGIN)
}

inline fun sqr(x: Double) = x * x

/*
 * Idea
 * Instead of double we could use 64bit fixed point arithmetic for velocities because all <1 anyway and
 * it has higher precision (multiplication is fast with Math.multiplyHigh()).
 *
 * Docs use MathJAX https://www.mathjax.org/#gettingstarted and ESVECT https://ctan.org/pkg/esvect
 */
