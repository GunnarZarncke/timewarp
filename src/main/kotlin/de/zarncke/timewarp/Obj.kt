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
class Obj(val name: String) {
    /**
     * There can only be one motion at a time; if no motion is specified the object continues in inertial
     * movement i.e. with the last velocity.
     */
    private val motions: TreeMap<Double, Motion> = TreeMap()

    /**
     * Any overlapping number of actions can be specified, they just can't change the movement of the object.
     */
    private val actions: TreeSet<Action<Any>> = TreeSet(object : Comparator<Action<Any>> {
        override fun compare(o1: Action<Any>?, o2: Action<Any>?) = compareValues(o1?.range(), o2?.range())
    })

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