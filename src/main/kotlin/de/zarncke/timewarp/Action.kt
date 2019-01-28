package de.zarncke.timewarp

import de.zarncke.timewarp.Action.RetrySmallerStep
import de.zarncke.timewarp.math.*

/**
 * Defines an abstract action that an [Obj] can perform at or during a time (measured in proper time).
 * An Action can
 */
abstract class Action<T>(val tauStart: Double, val tauEnd: Double = tauStart) {

    class RetrySmallerStep : Exception()

    abstract fun init(): T

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
     *
     * By default this method creates an event about its execution.
     *
     * @param world with state of all objects as seen by
     * @param obj acted on at
     * @param tau proper time of object
     * @throws RetrySmallerStep indicates that the simulation should use smaller steps to approximate an event
     */
    open fun act(world: WorldView, obj: Obj, tau: Double, t: T): T {
        world.addEvent(
            Event(
                "Action:${javaClass.simpleName}",
                world.stateInFrame(obj).r,
                obj,
                tau,
                obj,
                tau
            )
        )
        return t
    }

    fun range() = Range(tauStart, tauEnd)
}

/**
 * Creates a collision event if this object is at the same position as another (tracked) object.
 * Currently does *not* reduce time step if objects get close to another, but only detects collision at existing
 * events/simulation times.
 *
 * Only detects collisions during the specified interval (can be open-ended).
 */
open class DetectCollision(tau: Double, until: Double = Double.POSITIVE_INFINITY, val targets: Set<Obj>) :
    Action<DetectCollision.MyState>(tau, until) {
    class MyState(val generated: Set<Obj> = setOf<Obj>())

    override fun init() = MyState()


    override fun act(world: WorldView, obj: Obj, tau: Double, state: MyState): MyState {
        val sourcePos = world.stateInFrame(obj)
        val added = mutableSetOf<Obj>()
        val removed = mutableSetOf<Obj>()
        for (target in targets - state.generated) {
            val targetPos = world.stateInFrame(target)
            val dr = (targetPos.r.to3() - sourcePos.r.to3()).abs()
            // TODO if dr is small compared to time step reduce time step
            if (dr < eps * 2) {
                collide(world, obj, sourcePos, target, targetPos)
                added.add(target)
            } else if (dr > eps * 2) removed.add(target)
        }
        return MyState(state.generated + added - removed)
    }

    open fun collide(world: WorldView, self: Obj, selfPos: State, target: Obj, targetPos: State) {
        world.addEvent(Event("collide", selfPos.r, self, selfPos.tau, target, targetPos.tau))
    }
}

/**
 * A Sender action creates periodical [Pulse]s originating from its objects 4-vector.
 * The period is determined from object proper time.
 */
class Sender(val name: String, val start: Double, val period: Double, val no: Int = 0) : Action<Unit>(start, start) {
    override fun init() {
    }

    override fun act(world: WorldView, obj: Obj, tau: Double, t: Unit) {
        if (tau == start) {
            world.addAction(obj, Sender(name, start + period, period, no + 1))
            world.addAction(obj, Pulse("pulse:$name-$no", start))
        }
    }
}

/**
 * A Pulse is a single light signal that is sent at the start time (proper) in all directions and received by
 * all objects that it reaches, creating an Event at the intercept 4-vector.
 *
 * This works by
 * 1) tracking [Separation.SPACELIKE] separated objects
 * 2) reducing time-stap ([RetrySmallerStep]) when they overshoot

 */
class Pulse(val name: String, start: Double) : Action<Pulse.MyState>(start, Double.POSITIVE_INFINITY) {
    class MyState(
        val impossible: Set<Obj> = setOf<Obj>(),
        val tracked: Set<Obj> = setOf<Obj>()
    )

    override fun init()= MyState ()

    override fun act(world: WorldView, obj: Obj, tau: Double, state:MyState) :MyState{
        // note: sourcePos.t is transformed start tau
        // and objPos.t is transformed now tau
        val sourcePos = world.stateInFrame(obj)

        val impossible = state.impossible.toMutableSet()
        val tracked = state.tracked.toMutableSet()

        for (newObj in world.objects - impossible - tracked) {
            val newObjPos = world.stateInFrame(newObj)
            when (separation(newObjPos.r, sourcePos.r, eps)) {
                Separation.TIMELIKE -> impossible.add(newObj)
                Separation.LIGHTLIKE -> {// immediate hit
                    world.addEvent(
                        Event(
                            name,
                            newObjPos.r,
                            obj,
                            sourcePos.tau,
                            newObj,
                            newObjPos.tau
                        )
                    )
                    impossible.add(newObj)
                }
                Separation.SPACELIKE -> tracked.add(newObj)
            }
        }
        for (other in tracked) {
            val otherPos = world.stateInFrame(other)
            when (separation(otherPos.r, sourcePos.r, eps)) {
                Separation.TIMELIKE -> throw RetrySmallerStep() // overshot
                Separation.LIGHTLIKE -> {
                    world.addEvent(
                        Event(
                            name,
                            otherPos.r,
                            obj,
                            sourcePos.tau,
                            other,
                            otherPos.tau
                        )
                    )
                    tracked.remove(other)
                    impossible.add(other)
                }
                Separation.SPACELIKE -> Unit // wait
            }
        }
        return MyState(impossible, tracked)
    }
}
