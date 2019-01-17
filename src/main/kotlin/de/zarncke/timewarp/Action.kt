package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import kotlin.math.abs

/**
 * Defines an abstract action that an [Obj] can perform at or during a time (measured in proper time).
 * An Action can
 */
abstract class Action(val tauStart: Double, val tauEnd: Double = tauStart) {
    class RetrySmallerStep : Exception()

    data class Changes(
        val actions: MutableSet<Pair<Obj, Action>> = mutableSetOf(),
        val motions: MutableList<Pair<Obj, Motion>> = mutableListOf(),
        val objects: MutableSet<Pair<Obj, Vector4>> = mutableSetOf(),
        val events: MutableSet<TimeWarp.Event> = mutableSetOf()
    ) {
        fun applyChanges(world: TimeWarp.World) {
            for (entry in actions) {
                entry.first.addAction(entry.second)
            }
            for (entry in motions) {
                entry.first.addMotion(entry.second)
            }
            for (entry in objects) {
                world.addObj(entry.first)
                world.set(entry.first, State(entry.second, V3_0, 0.0))
            }
            world.events.addAll(events)
        }
    }


    /**
     * This method will be called at the proper time tauStart and tauEnd.
     * It may also be called at other times in between (at other events at the same coordinate time).
     * when of the associated object is greater or equal the given time.
     * If the Action needs to narrow down on a certain time it may throw [RetrySmallerStep] which will cause
     * time interval halving. Caution: Slow.
     * <br/>
     * The method may...
     * <ul>
     *     <li>inspect the World but not change it (instead use changes)</li>
     *     <li>return new actions in the future or now</li>
     *     <li>return new motions in the future or now (but no overlapping motions are allowed)</li>
     *     <li>return new objects in the lightcone of this object</li>
     *     <li>throw [RetrySmallerStep] to indicate more detailed processing (see above)</li>
     * </ul>
     * @param world with state of all objects as seen by
     * @param obj acted on at
     * @param tau proper time of object
     * @param changes to use
     * @throws RetrySmallerStep indicates that the simulation should use smaller steps to approximate an event
     */
    open fun act(world: TimeWarp.World, obj: Obj, tau: Double, changes: Changes) {}

    fun range() = Range(tauStart, tauEnd)
}

/**
 * Creates a collision event if this object is at the same position as another (tracked) object.
 * Currently does *not* reduce time step if objects get close to another, but only detects collision at existing
 * events/simulation times.
 */
open class DetectCollision(tau: Double, val until: Double, val targets: Set<Obj>) :
    Action(tau, Double.POSITIVE_INFINITY) {
    private val generated = mutableSetOf<Obj>()
    override fun act(world: TimeWarp.World, obj: Obj, tau: Double, changes: Changes) {
        val sourcePos = world.stateInFrame(obj, world.origin)
        for (target in targets - generated) {
            val targetPos = world.stateInFrame(target, world.origin)
            val dr = (targetPos.r.to3() - sourcePos.r.to3()).abs()
            // TODO if dr is small compared to time step reduce time step
            if (dr < eps) {
                collide(changes, obj, sourcePos, target, targetPos)
                generated.add(target)
            } else if (dr > eps) generated.remove(target)
        }
    }

    open fun collide(changes: Changes, self: Obj, selfPos: State, target: Obj, targetPos: State) {
        changes.events.add(TimeWarp.Event("collide", selfPos.r, self, selfPos.tau, target, targetPos.tau))
    }
}

/**
 * A Sender action creates periodical [Pulse]s originating from its objects 4-vector.
 * The period is determined from object proper time.
 */
class Sender(val name: String, val start: Double, val period: Double, val no: Int = 0) : Action(start, start) {
    override fun act(world: TimeWarp.World, obj: Obj, tau: Double, changes: Changes) {
        if (tau == start) {
            changes.actions.add(obj to Sender(name, start + period, period, no + 1))
            changes.actions.add(obj to Pulse("pulse:$name-$no", start))
        }
    }
}

/**
 * A Pulse is a single light signal that is sent at the start time (proper) in all directions and received by
 * all objects that it reaches, creating an Event at the intercept 4-vector.
 */
class Pulse(val name: String, start: Double) : Action(start, Double.POSITIVE_INFINITY) {
    private val impossible = mutableSetOf<Obj>()
    private val tracked = mutableSetOf<Obj>()

    override fun act(world: TimeWarp.World, obj: Obj, tau: Double, changes: Changes) {
        // note: sourcePos.t is transformed start tau
        // and objPos.t is transformed now tau
        val sourcePos = world.stateInFrame(obj, world.origin)

        for (newObj in world.objects - impossible - tracked) {
            val newObjPos = world.stateInFrame(newObj, world.origin)
            when (separation(newObjPos.r, sourcePos.r, eps)) {
                Separation.TIMELIKE -> impossible.add(newObj)
                Separation.LIGHTLIKE -> {// immediate hit
                    changes.events.add(TimeWarp.Event(name, newObjPos.r, obj, sourcePos.tau, newObj, newObjPos.tau))
                    impossible.add(newObj)
                }
                Separation.SPACELIKE -> tracked.add(newObj)
            }
        }
        for (other in tracked) {
            val otherPos = world.stateInFrame(other, world.origin)
            when (separation(otherPos.r, sourcePos.r, eps)) {
                Separation.TIMELIKE -> throw RetrySmallerStep() // overshot
                Separation.LIGHTLIKE -> {
                    changes.events.add(TimeWarp.Event(name, otherPos.r, obj, sourcePos.tau, other, otherPos.tau))
                    tracked.remove(other)
                    impossible.add(other)
                }
                Separation.SPACELIKE -> Unit // wait
            }
        }
    }
}
