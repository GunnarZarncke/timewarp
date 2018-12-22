package de.zarncke.timewarp

import java.util.*

class TimeWarp {
    private var origin = Space(V4_0, V3_0)

    val space: Space
        get() = origin

    var now: Double = 0.0

    val objects: MutableList<Obj> = mutableListOf()

    fun init() {
        addObj(Obj(origin))
    }

    fun addObj(obj: Obj) {
        objects.add(obj)
    }

    fun simulateTo(t: Double) {
        val allLater = objects.map { obj -> obj.actions.map { it.t to (obj to it) } }.flatten().toMap().toSortedMap()
            .subMap(now, t)
        allLater.values.forEach { (obj, action) -> action.act(obj,t) }

        now = t
    }

    fun position(obj: Obj): Vector4 {
        return obj.pos.to4(now)
    }

}

class Space(val displacement: Vector4, val velocity: Vector3) {

}

val V3_0 = Vector3(0.0, 0.0, 0.0)
val V4_0 = Vector4(0.0, 0.0, 0.0, 0.0)

data class Vector3(val x: Double, val y: Double, val z: Double) {
    fun to4(t: Double) = Vector4(t, x, y, z)
    operator fun times(d: Double): Vector3 = Vector3(x*d,y*d,z*d)
    operator fun plus(v2: Vector3): Vector3 = Vector3(x+v2.x,y+v2.y,z+v2.y)
    operator fun minus(v2: Vector3): Vector3 = Vector3(x-v2.x,y-v2.y,z-v2.y)
}

data class Vector4(val t: Double, val x: Double, val y: Double, val z: Double) {
    fun to3() = Vector3(x, y, z)
    //operator fun times(d: Double): Vector4 = Vector4(t*d, x*d,y*d,z*d)
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

open class Action(val t: Double) : Comparable<Action> {
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
