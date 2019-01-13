package de.zarncke.timewarp

import java.lang.IllegalArgumentException
import java.util.*

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
    private val actions: TreeSet<Action> = TreeSet()

    fun motions(): SortedMap<Double, Motion> = motions
    fun actions(): Set<Action> = actions

    fun addMotion(motion: Motion) {
        val overlaps = motions.subMap(motion.tauStart, false, motion.tauEnd, false)
        if (!overlaps.isEmpty())
            throw IllegalArgumentException("motion $motion overlaps with $overlaps")
        val next = motions.tailMap(motion.tauEnd).values.firstOrNull()
        if (next != null && motion.tauEnd > next.tauStart)
            throw IllegalArgumentException("motion $motion overlaps partly with $next")
        motions.put(motion.tauStart, motion)
    }

    fun addAction(action: Action) {
        if (action.tauEnd < action.tauStart) throw IllegalArgumentException()
        actions.add(action)
    }
}