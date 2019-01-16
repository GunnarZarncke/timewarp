package de.zarncke.timewarp

import de.zarncke.timewarp.math.*

abstract class Motion(val tauStart: Double, val tauEnd: Double) {

    /**
     * Determines the location and velocity of an object at a given <em>proper time</em> tauTo within a co-moving reference frame.
     * This method will be called when the proper time of the associated object is greater or equal the motion start time.
     * It may be called multiple times with different values (or even the same value) for tauNow and tauTo (in corresponding reference frames).
     * @param obj in
     * @param tauNow proper time of co-moving object (sometime during the motion) to
     * @param tauTo proper time of object (also during motion, but may be *before* tauNow)
     * @return state (4-vector and velocity of object at proper time tau within given frame)
     */
    abstract fun moveUntilProperTime(coMovingFrame: Frame, tauNow: Double, tauTo: Double): State

    // TODO check what happens with lateral acceleration, maybe this coMovingFrame thing is not yet well thought out

    /**
     * Determines the state (location,  velocity proper time) of an object in the given reference frame
     * at a given <em>coordinate time</em> in the world frame.
     * @param coMovingFrame or the moving object at time tauStart (!)
     * @param t coordinate time of object in world frame
     * @return state (4-vector and velocity and tau of object either at t or the end of the motion whatever is earlier)
     */
    abstract fun moveUntilCoordinateTime(coMovingFrame: Frame, t: Double): State

    override fun toString() = "${javaClass.simpleName}($tauStart-$tauEnd)"

}

/**
 * Inertial motion stays at the origin of its comoving reference frame.
 */
open class Inertial(tauStart: Double, tauEnd: Double) : Motion(tauStart, tauEnd) {
    override fun moveUntilProperTime(coMovingFrame: Frame, tauNow: Double, tauTo: Double): State {
        return State(V3_0.to4(tauTo - tauNow), V3_0, tauTo)
    }

    override fun moveUntilCoordinateTime(coMovingFrame: Frame, t: Double): State {
        val dt = t - coMovingFrame.r.t
        assert(dt >= 0)
        var tau = dt/gamma(coMovingFrame.v.abs())
        if (this.tauStart + tau > tauEnd) tau = tauEnd - tauStart
        return State(V3_0.to4(tau), V3_0, this.tauStart + tau).transform(coMovingFrame, Frame.ORIGIN)
    }
}

/**
 * Instantly (at tauStart) changes motion to given relative velocity relative to its comoving reference frame.
 * (we disregard that this requires unphysical infinite acceleration and assume that it is "a very short very strong acceleration")
 * @param tauStart time of the velocity change
 * @param v new velocity as measured in a reference frame prior to the velocity change
 */
class AbruptVelocityChange(tauStart: Double, val v: Vector3) : Inertial(tauStart, tauStart) {
    override fun moveUntilProperTime(coMovingFrame: Frame, tauNow: Double, tauTo: Double): State {
        assert(tauStart == tauNow)
        assert(tauNow == tauTo)
        return State(V4_0, v, tauNow)
    }

    override fun moveUntilCoordinateTime(coMovingFrame: Frame, t: Double): State {
        return State(V4_0, v, tauStart).transform(coMovingFrame, Frame.ORIGIN)
    }

    override fun toString() = "${super.toString()} v=$v"
}

/**
 * "a) Hyperbolic motion: The constant, longitudinal proper acceleration
 * {\alpha =a_{x}^{0}=a_{x}\gamma ^{3}} by (4a) leads to the world line..."
 * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)#Curved_world_lines
 */
class LongitudinalAcceleration(tauStart: Double, tauEnd: Double, val a: Vector3) : Motion(tauStart, tauEnd) {
    override fun moveUntilProperTime(coMovingFrame: Frame, tauNow: Double, tauTo: Double): State {
        return relativisticAcceleration(a, tauTo - tauNow).copy(tau = tauTo)
    }

    override fun moveUntilCoordinateTime(coMovingFrame: Frame, t: Double): State {
        val state = relativisticCoordAcceleration(a, t, coMovingFrame)

        if (this.tauStart + state.tau < tauEnd)
            return state.copy(tau = state.tau + tauStart)

        // if the determined proper time doesn't fall into the range return a state up to the end
        return relativisticAcceleration(a, tauEnd - tauStart).copy(tau = tauEnd).transform(coMovingFrame, Frame.ORIGIN)
    }

    override fun toString() = "${super.toString()} a=$a"
}