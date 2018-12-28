package de.zarncke.timewarp

import java.lang.IllegalArgumentException
import java.util.*
import kotlin.math.cosh
import kotlin.math.sinh
import kotlin.math.sqrt
import kotlin.math.tanh

/**
 * Relativistic simulation.
 *
 * Velocity v with unit c=1
 */
class TimeWarp {
    // Koma: mat[1.0,1.0,1.0].T*mat[1.0,1.0,1.0]

    companion object {
        val V3_0 = Vector3(0.0, 0.0, 0.0)
        val V4_0 = Vector4(0.0, 0.0, 0.0, 0.0)

    }

    class World {
        var origin = Frame(V4_0, V3_0)

        val frame: Frame
            get() = origin

        var now: Double = 0.0

        val objects: MutableList<Obj> = mutableListOf()
        val events: MutableList<Event> = mutableListOf()
        fun addObj(obj: Obj) {
            objects.add(obj)
        }
    }

    val world = World()
    val formula = Formula()

    fun init() {
    }

    fun addObj(obj: Obj, position: Vector4) {
        obj.position = position
        world.addObj(obj)
    }


    fun simulateTo(t: Double):World {
        val allLater =
            world.objects.map { obj -> obj.actions.map { it.t to (obj to it) } }.flatten().toMap().toSortedMap()
                .subMap(world.now, t)
        allLater.values.forEach { (obj, action) -> action.act(obj, t) }

        world.now = t
        return world
    }

    class Frame(val displacement: Vector4, val velocity: Vector3) {

    }

    class Obj(val name: String) {
        var position: Vector4 = V4_0

        val actions: TreeSet<Action> = TreeSet()

        fun addAction(action: Action) {
            actions.add(action)
        }

        fun positionInFrame(s: Frame): Vector4 {
            return s.displacement + lorentzTransform(s.velocity, position)
        }
    }

    abstract class Action(val t: Double) : Comparable<Action> {
        override fun compareTo(other: Action) = t.compareTo(other.t)
        open fun act(obj: Obj, at: Double) {}
    }

    class Motion(t: Double, val to: Vector4) : Action(t) {
        override fun act(obj: Obj, at: Double) {
            obj.position = (to.to3() * (at - t)).to4(at)
        }
    }
    class Accelerate(t: Double, val a: Vector3, val tEnd:Double) : Action(t) {
        override fun act(obj: Obj, at: Double) {
            // TODO
        }
    }

    class Pulse(val name: String, start: Double, period: Double): Action(start)

    class Clock(val frame: Frame, var tau: Double)

    data class Event(
        val name: String,
        val position: Vector4,
        val sender: Obj,
        val receiver: Obj
    )

    interface Actor {
        fun onEvent(e: Event, world: World, api: Api)
    }

    interface Api {
        fun addAction(a: Action)
        fun addPulse(name: String)
        fun addObject(obj: Obj)
    }

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
 * https://en.wikipedia.org/wiki/Lorentz_transformation#Vector_transformations
 * @param v as velocity of a second frame S′, as measured in S
 * @param r as 4-vector of an event within a Lorentz frame S (aka Frame)
 * @return r' as the transformed 4-vector of the event within the second frame.
 */
fun lorentzTransform(v: Vector3, r: Vector4): Vector4 {
    val vAbs = v.abs()
    if (vAbs == 0.0) return r
    val n = v * (1 / vAbs)
    val r3 = r.to3()
    val gamma = gamma(vAbs)
    return Vector4(
        gamma * (r.t - vAbs * n.dot(r3)),
        r3 + n * ((gamma - 1) * r3.dot(n)) - n * (gamma * r.t * vAbs)
    )
}

/**
 * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)
 * http://math.ucr.edu/home/baez/physics/Relativity/SR/Rocket/rocket.html
 * @param a0 proper acceleration in (momentarily co-moving) reference frame
 * @param tau proper time duration of the acceleration of the accelerated object
 * @return 4-vector of the resulting position within the original reference frame x
 *  velocity at proper time tau within the original reference frame
 */
fun relativisticAcceleration(a0: Vector3, tau: Double): Pair<Vector4, Vector3> {
    val aAbs = a0.abs()
    if (aAbs == 0.0) return TimeWarp.V3_0.to4(tau) to TimeWarp.V3_0
    val n = a0 * (1 / aAbs)
    return Vector4(sinh(aAbs * tau) / aAbs, n * ((cosh(aAbs * tau) - 1) / aAbs)) to
            n * tanh(aAbs * tau)
}
