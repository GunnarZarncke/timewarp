package de.zarncke.timewarp

import java.lang.IllegalArgumentException
import java.util.*
import kotlin.Exception
import kotlin.math.*

/**
 * Relativistic simulation.
 *
 * Velocity v with unit c=1
 */
class TimeWarp {
    // Koma: mat[1.0,1.0,1.0].T*mat[1.0,1.0,1.0]

    class World(
        var now: Double = 0.0,
        val objects: MutableList<Obj> = mutableListOf(),
        val events: MutableList<Event> = mutableListOf(),
        val states: MutableMap<Obj, Pair<Vector4, Vector3>> = mutableMapOf()
    ) {
        var origin = Frame(V4_0, V3_0)

        fun addObj(obj: Obj) {
            objects.add(obj)
        }

        fun comovingFrame(obj: Obj) =
            (states[obj] ?: throw IllegalArgumentException("unknown object $obj"))
                .let { Frame(it.first, it.second) }

        /**
         * @param obj to get position and velocity for (at current time t in world frame)
         * @param s frame to get state in
         * @return state (position and velocity) in the given frame
         */
        fun stateInFrame(obj: Obj, s: Frame) =
            (states[obj] ?: throw IllegalArgumentException("unknown object $obj")).let {
                lorentzTransform(s.velocity, it.first - s.displacement) to
                        transformedAddedVelocity(s.velocity, it.second)
            }

        fun set(obj: TimeWarp.Obj, position: Vector4 = V4_0, velocity: Vector3 = V3_0) {
            states[obj] = position to velocity
        }

        fun copy() = World(now, objects.toMutableList(), events.toMutableList(), states.toMutableMap())
    }

    val world = World()
    val formula = Formula()

    fun init() {
    }

    fun addObj(obj: Obj, position: Vector4) {
        world.addObj(obj)
        world.set(obj, position, V3_0)
    }


    fun simulateTo(t: Double): World {
        // repeat taking the next event and processing it until the target time has been reached
        // TODO proper time of action cannot be transformed into common time -> use multiple queues (?)
        // -> events will be chronological (causal) in any reference frame
        // TODO inertial motion: without explicit Acceleration objects should continue moving along
        while (true) {
            // for each object
            //   among movements and unmarked actions not earlier than now
            //     determine proper time tau of first scheduled action
            //     determine all scheduled motions up to that event (might require creation of ad-hoc inertial moves)
            //     simulate all motions up to tau
            //     calculate 4-vector (in world frame) of the object at tau
            // take the earliest of these 4-vectors call it r and its object o and action a
            // for each object
            //   execute motion to r (see below) determining tau_o
            //   update the object in the world to it's 4-vector
            // execute action a on o with the world and tau_o as parameter (may update events)
            // mark a as done
            //
            // execute motion of obj to r
            //   set p = current position of obj
            //   set t = now
            //   while t<r.t
            //     take motion at current time, if there is none create inertial up the next one
            //     calculate q and tau for r.t with current motion
            //     if tau > next motion
            //       set p = move to tau
            //       set t = q.t
            //     else
            //       return q

            /*
            val (start, objAct) =
                    world.objects.map { obj ->
                        obj.actions.map {
                            lorentzTransformInv(
                                world.stateInFrame(obj, world.origin),
                                it.tau
                            ).t to (obj to it)
                        }
                    }
                        .flatten()
                        .toMap().toSortedMap()
                        .subMap(world.now, t).entries.firstOrNull()
                        ?: break
            val (obj, action) = objAct
            action.act(world, obj, t)
            val pos = world.stateInFrame(obj, world.origin)
            world.events.add(Event("Action:$action", pos, obj, obj))
            */
        }

        return world
    }

    class Frame(val displacement: Vector4, val velocity: Vector3) {

    }

    class Obj(val name: String) {
        /**
         * There can only be one motion at a time; if no motion is specified the object continues in inertial
         * movement i.e. with the last velocity.
         */
        val motions: TreeSet<Motion> = TreeSet()

        /**
         * Any overlapping number of actions can be specified, they just can't change the movement of the object.
         */
        val actions: TreeSet<Action> = TreeSet()

        fun addMotion(motion: Motion) {
            motions.add(motion)
        }

        fun addAction(action: Action) {
            actions.add(action)
        }
    }

    class RetrySmallerStep : Exception()

    abstract class Motion(val tauStart: Double) {
        /**
         * Determines the location and velocity of an object at a given <em>proper time</em> tau within a co-moving reference frame.
         * This method will be called when the proper time of the associated object is greater or equal the motion time.
         * It may be called multiple times with different values (or even the same value) for tau.
         * @param obj to be moved up to
         * @param tau proper time of object in
         * @param coMovingFrame co-moving with object at time tauStart
         * @return 4-vector of object at proper time tau within given frame x
         *  relative velocity at that time in the given frame
         */
        abstract fun moveToProperTime(obj: Obj, tau: Double, coMovingFrame: Frame): Pair<Vector4, Vector3>

        /**
         * Determines the location and velocity of an object at a given <em>coordinate time</em> t within the given reference frame.
         * This method will be called to determine the location and proper time of the associated object at coordinate time.
         * @param obj at
         * @param t coordinate time of object in
         * @param inertialFrame co-moving with object at time tauStart
         * @return 4-vector of object at tau within given frame x
         *   relative velocity at tau in the given frame x
         *   time tau of the end of the motion (either corresponding to t or the end of the motion)
         */
        abstract fun moveToObserverTime(obj: Obj, t: Double, inertialFrame: Frame): Pair<Pair<Vector4, Vector3>, Double>
    }

    class Inertial(tauStart: Double, val v: Vector3, val tauEnd: Double) : Motion(tauStart) {
        override fun moveToProperTime(obj: Obj, tau: Double, coMovingFrame: Frame): Pair<Vector4, Vector3> {
            return (v * (tau - this.tauStart)).to4(tau) to v
        }

        override fun moveToObserverTime(
            obj: Obj,
            t: Double,
            inertialFrame: Frame
        ): Pair<Pair<Vector4, Vector3>, Double> {
            // t = tau for inertial motion
            // see https://en.wikipedia.org/wiki/Proper_time
            var dt = (t - inertialFrame.displacement.t)
            if (this.tauStart + dt > tauEnd) dt = tauEnd - tauStart
            val p = (v * dt).to4(tauStart + dt)
            return p to v to this.tauStart + dt
        }
    }

    /**
     * @param tau proper time of start of acceleration
     * @param a proper acceleration ("alpha") in momentarily co-moving reference frame
     * @param tauEnd proper end time of motion
     */
    class Accelerate(tau: Double, val a: Vector3, val tauEnd: Double) : Motion(tau) {
        override fun moveToProperTime(obj: Obj, tau: Double, coMovingFrame: Frame): Pair<Vector4, Vector3> {
            return relativisticAcceleration(a, tau - this.tauStart)
        }

        override fun moveToObserverTime(
            obj: Obj,
            t: Double,
            inertialFrame: Frame
        ): Pair<Pair<Vector4, Vector3>, Double> {
            var dt = (t - inertialFrame.displacement.t)
            val pvtau = relativisticCoordAcceleration(a, dt)

            if (this.tauStart + pvtau.second > tauEnd) return relativisticAcceleration(a, tauEnd - tauStart) to tauEnd
            return pvtau
        }
    }

    abstract class Action(val tau: Double) {

        /**
         * This method will be called when the proper time of the associated object is greater or equal the given time.
         * It may be called multiple times with different values (or even the same value) for tau.
         * The method may do
         * <ul>
         *     <li>Add new actions in the future or now</li>
         *     <li>Add new motions in the future or now (but no overlapping motions are allowed)</li>
         *     <li>Add new objects to the world (but only in the lightcone of this object)</li>
         * </ul>
         * @param world with state of all objects as seen by
         * @param obj acted on at
         * @param tau proper time of object
         * @throws RetrySmallerStep indicates that the simulation should use smaller steps to approximate an event
         */
        open fun act(world: World, obj: Obj, tau: Double) {}
    }

    /**
     * Creates a collision event if this object is at the same position as another (tracked) object.
     * Currently does *not* reduce time step if objects get close to another, but only detects collision at existing
     * events/simulation times.
     */
    open class DetectCollision(tau: Double, val until: Double, val targets: Set<Obj>) : Action(tau) {
        val generated = mutableSetOf<Obj>()
        override fun act(world: World, self: Obj, tau: Double) {
            val sourcePos = world.stateInFrame(self, world.origin)
            for (obj in targets - generated) {
                val objPos = world.stateInFrame(obj, world.origin)
                val dr = (objPos.first.to3() - sourcePos.first.to3()).abs()
                // TODO if dr is small compared to time step reduce time step
                if (dr < eps) {
                    collide(world, objPos, self, obj)
                    generated.add(obj)
                } else if (dr > eps) generated.remove(obj)
            }
        }

        open fun collide(world: World, objPos: Pair<Vector4, Vector3>, self: Obj, obj: Obj) {
            world.events.add(Event("collide", objPos.first, self, obj))
        }
    }

    /**
     * A Sender action creates periodical [Pulse]s originating from its objects 4-vector.
     * The period is determined from object proper time.
     */
    class Sender(val name: String, val start: Double, val period: Double, val no: Int = 0) : Action(start) {
        override fun act(world: World, obj: Obj, tau: Double) {
            if (tau == start) {
                obj.actions.add(Sender(name, start + period, period, no + 1))
                obj.actions.add(Pulse("pulse:$name-$no", start))
            }
        }
    }

    /**
     * A Pulse is a single light signal that is sent at the start time (proper) in all directions and received by
     * all objects that it reaches, creating an Event at the intercept 4-vector.
     */
    class Pulse(val name: String, start: Double) : Action(start) {
        private val generated = mutableSetOf<Obj>()
        override fun act(world: World, obj: Obj, tau: Double) {
            // note: sourcePos.t is transformed start tau
            // and objPos.t is transformed now tau
            val sourcePos = world.stateInFrame(obj, world.origin).first
            for (other in world.objects - generated) {
                val objPos = world.stateInFrame(other, world.origin).first
                val dr = (objPos.to3() - sourcePos.to3()).abs()
                val dt = objPos.t - sourcePos.t
                if (abs(dr - dt) < eps) {
                    world.events.add(Event(name, objPos, obj, other))
                    generated.add(other)
                } else if (dr > dt) throw RetrySmallerStep()
            }
        }
    }

    data class Event(
        val name: String,
        val position: Vector4,
        val sender: Obj,
        val receiver: Obj
    )

    class Formula {

        val variables = mutableSetOf<String>()
        val equations = mutableListOf<String>()
        var v1 = 0
        var v2 = 0
        var t = 0

        fun varForV1(): String {
            val v = "v_$v1"; v1++; variables.add(v); return v
        }

        fun varForV2(): String {
            val v = "u_$v2"; v2++; variables.add(v); return v
        }

        fun varForTime(): String {
            val v = "t_$t"; t++; variables.add(v); return v
        }

        fun addVelocity(v: String, u: String): String {
            val w = varForV1()
            equations.add(
                "$w &= $v \\oplus  $u &= " +
                        "\\frac{1}{1 - $u \\cdot $v}\\left[$u - $v + \\frac{\\gamma_$v}{1+\\gamma_$v} $v \\times($v \\times $u)\\right]"
            )
            return w
        }

    }
}

// TODO move into context object?
var eps = 0.000001

val V3_0 = Vector3(0.0, 0.0, 0.0)
val V4_0 = Vector4(0.0, 0.0, 0.0, 0.0)

data class Vector3(val x: Double, val y: Double, val z: Double) {
    fun to4(t: Double) = Vector4(t, x, y, z)
    operator fun times(d: Double) = Vector3(x * d, y * d, z * d)
    operator fun plus(v2: Vector3) = Vector3(x + v2.x, y + v2.y, z + v2.y)
    operator fun minus(v2: Vector3) = Vector3(x - v2.x, y - v2.y, z - v2.y)
    fun abs() = sqrt(x * x + y * y + z * z)
    fun dot(d: Vector3) = x * d.x + y * d.y + z * d.z
    fun cross(d: Vector3) = Vector3(y * d.z - z * d.y, z * d.x - x * d.z, x * d.y - y * d.x)
}

data class Vector4(val t: Double, val x: Double, val y: Double, val z: Double) {
    fun to3() = Vector3(x, y, z)
    operator fun plus(v2: Vector4) = Vector4(t + v2.t, x + v2.x, y + v2.y, z + v2.y)
    operator fun minus(v2: Vector4) = Vector4(t + v2.t, x + v2.x, y + v2.y, z + v2.y)

    constructor(t: Double, v3: Vector3) : this(t, v3.x, v3.y, v3.z)
}

private fun gamma2(v: Double) = 1 / sqrt(1 - v)
/**
 * The gamma function is the relativistic Lorentz factor.
 * @param v velocity
 * @return lorentzFactor
 */
fun gamma(v: Double) = gamma2(v * v)

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
 * @return 4-vector of the resulting position within the original reference frame x
 *  velocity at proper time tau within the original reference frame
 */
fun relativisticAcceleration(a0: Vector3, tau: Double): Pair<Vector4, Vector3> {
    val aAbs = a0.abs()
    if (aAbs == 0.0) return V3_0.to4(tau) to V3_0
    val n = a0 * (1 / aAbs)
    return Vector4(sinh(aAbs * tau) / aAbs, n * ((cosh(aAbs * tau) - 1) / aAbs)) to
            n * tanh(aAbs * tau)
}

/**
 * Determine location of accelerated motion for coordinate time.
 * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)
 * @param a0 proper acceleration in (momentarily co-moving) reference frame
 * @param tau proper time duration of the acceleration of the accelerated object
 * @return 4-vector of the resulting position within the original reference frame x
 *  velocity at proper time tau within the original reference frame
 */
fun relativisticCoordAcceleration(a0: Vector3, t: Double): Pair<Pair<Vector4, Vector3>, Double> {
    val aAbs = a0.abs()
    if (aAbs == 0.0) return V3_0.to4(t) to V3_0 to t
    val v = aAbs * t
    val tau = ln(sqrt(1.0 + v.pow(2)) + v) / aAbs
    val n = a0 * (1 / aAbs)
    return Vector4(t, n * ((sqrt(1 + v.pow(2)) - 1) / aAbs)) to n * v to tau
}

