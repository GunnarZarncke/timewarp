package de.zarncke.timewarp

import de.zarncke.timewarp.math.Range
import java.lang.IllegalArgumentException
import java.util.*
import kotlin.Comparator

/**
 * An Obj lives in a [World] where it moves via [Motion]s and acts via [Action]s.
 * An Obj has a [TimeWarp.World.stateInFrame|state] in the world i.e. position and time in that world.
 * This includes the proper time on a clock moving with the object.
 */
class Obj(override val name: String):Cause<Obj> {
    override fun compareTo(other: Obj) = compareValues(name, other.name)

    /**
     * There can only be one motion at a time; if no motion is specified the object continues in inertial
     * movement i.e. with the last velocity.
     */
    private val motions: TreeMap<Double, Motion> = TreeMap()

    /**
     * Any overlapping number of actions can be specified, they just can't change the movement of the object
     * (except by changing the uniqe sequence of motions).
     */
    private val actions: TreeSet<Action<Any>> = TreeSet(compareBy(Action<*>::range, Action<*>::name))

    fun motions(): SortedMap<Double, Motion> = motions
    fun actions(): Set<Action<Any>> = actions

    fun addMotion(motion: Motion) :Obj {
        val overlaps = motions.subMap(motion.tauStart, false, motion.tauEnd, false)
        if (!overlaps.isEmpty())
            throw IllegalArgumentException("motion $motion overlaps with $overlaps")
        val next = motions.tailMap(motion.tauEnd).values.firstOrNull()
        if (next != null && motion.tauEnd > next.tauStart)
            throw IllegalArgumentException("motion $motion overlaps partly with $next")
        motions.put(motion.tauStart, motion)
        return this
    }

    fun <T>addAction(action: Action<T>) : Obj{
        if (action.tauEnd < action.tauStart) throw IllegalArgumentException()
        actions.add(action as Action<Any>)
        return this
    }

    override fun toString(): String {
        return "$name[${javaClass.simpleName}]"
    }
}