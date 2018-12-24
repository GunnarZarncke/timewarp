package de.zarncke.timewarp

import koma.mat
import java.lang.IllegalArgumentException
import java.util.*
import kotlin.math.sqrt

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

        fun gamma2(v: Double) = 1 / sqrt(1 - v)
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
            if(vAbs == 1.0) throw IllegalArgumentException("spaces cannot move with lightspeed")
            val a = uPrime.dot(v)
            val gamma_v = gamma(vAbs)
            return (uPrime * (1.0 / gamma_v) + v + v * a * (gamma_v / (1 + gamma_v))) * (1.0 / (1.0 + a))
        }

        /**
         * ...u as velocity of a body within a Lorentz frame S, and v as velocity of a second frame S′, as measured in S,
         * and u′ as the transformed velocity of the body within the second frame.
         * https://en.wikipedia.org/wiki/Velocity-addition_formula
         * http://math.ucr.edu/home/baez/physics/Relativity/SR/velocity.html
         * @param u as velocity of a body within a Lorentz frame S (aka Space)
         * @param v as velocity of a second frame S′, as measured in S
         * @return u' as the transformed velocity of the body within the second frame.
         */
        fun transformedAddedVelocity(v: Vector3, u: Vector3): Vector3 {
            // we do not use the cross product form because I'm not sure it's not an approximation (and more ops)
            val vAbs = v.abs()
            if(vAbs == 1.0) throw IllegalArgumentException("spaces cannot move with lightspeed")
            val a = u.dot(v)
            val gamma_v = gamma(v.abs())
            return (u * (1.0 / gamma_v) - v + v * a * (gamma_v / (1 + gamma_v))) * (1.0 / (1.0 - a))
        }
    }


    private var origin = Space(V4_0, V3_0)

    val space: Space
        get() = origin

    var now: Double = 0.0

    val objects: MutableList<Obj> = mutableListOf()

    val formula = Formula()

    fun init() {
        addObj(Obj(origin))
    }

    fun addObj(obj: Obj) {
        objects.add(obj)
    }

    fun simulateTo(t: Double) {
        val allLater = objects.map { obj -> obj.actions.map { it.t to (obj to it) } }.flatten().toMap().toSortedMap()
            .subMap(now, t)
        allLater.values.forEach { (obj, action) -> action.act(obj, t) }

        now = t
    }

    fun position(obj: Obj): Vector4 {
        return obj.pos.to4(now)
    }

    class Space(val displacement: Vector4, val velocity: Vector3) {

    }

    class Obj {
        var space: Space
        var clock: Clock
        var pos: Vector3 = V3_0

        val actions: TreeSet<Action> = TreeSet()

        constructor(space: Space) {
            this.space = space
            clock = Clock(space)
        }

        fun addAction(action: Action) {
            actions.add(action)
        }
    }

    abstract class Action(val t: Double) : Comparable<Action> {
        override fun compareTo(other: Action) = t.compareTo(other.t)
        open fun act(obj: Obj, at: Double) {}
    }

    class Motion(t: Double, val to: Vector4) : Action(t) {
        override fun act(obj: Obj, at: Double) {
            obj.pos = to.to3() * (at - t)
        }
    }

    class Clock(val space: Space)

    class Event

    class Sender

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
    operator fun times(d: Double): Vector3 = Vector3(x * d, y * d, z * d)
    operator fun plus(v2: Vector3): Vector3 = Vector3(x + v2.x, y + v2.y, z + v2.y)
    operator fun minus(v2: Vector3): Vector3 = Vector3(x - v2.x, y - v2.y, z - v2.y)
    fun abs() = sqrt(x * x + y * y + z * z)
    fun dot(d: Vector3) = x * d.x + y * d.y + z * d.z
    fun cross(d: Vector3) = Vector3(y * d.z - z * d.y, z * d.x - x * d.z, x * d.y - y * d.x)
}

data class Vector4(val t: Double, val x: Double, val y: Double, val z: Double) {
    fun to3() = Vector3(x, y, z)
    //operator fun times(d: Double): Vector4 = Vector4(t*d, x*d,y*d,z*d)
}

