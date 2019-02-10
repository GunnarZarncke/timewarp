package de.zarncke.timewarp

import de.zarncke.timewarp.Action.RetrySmallerStep
import de.zarncke.timewarp.math.*
import java.lang.IllegalArgumentException
import java.lang.Math.abs

/**
 * Defines an abstract action that an [Obj] can perform at or during a time (measured in proper time).
 */
abstract class Action<T>(val tauStart: Double, val tauEnd: Double = tauStart) : Cause<Action<T>> {

    init {
        if (!tauStart.isFinite())
            throw IllegalArgumentException("start ${tauStart} must be finite")
        if (tauStart > tauEnd)
            throw IllegalArgumentException("start ${tauStart} may not be after end $tauEnd")
    }

    class RetrySmallerStep(val tHint: Double? = null) : Exception()

    override val name get() = javaClass.simpleName
    override val isSilent: Boolean get() = false
    override fun compareTo(other: Action<T>) = compareValues(name, other.name)

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
     *     <li>return new actions in the future of now (but att actions must be consistent)</li>
     *     <li>return new motions in the future of now (but no overlapping motions are allowed)</li>
     *     <li>return new objects in the lightcone of this object</li>
     *     <li>throw [RetrySmallerStep] to indicate more detailed processing (see above)</li>
     * </ul>
     *
     * By default this method creates an event about its execution.
     *
     * @param world with state of all objects as seen by
     * @param obj acted on at
     * @param tau proper time of object (guaranteed to be called at approximately startTau and endTau)
     * @param state of the action (if the action has no state use Unit)
     * @throws RetrySmallerStep indicates that the simulation should use smaller steps to approximate an event
     */
    open fun act(world: WorldView, obj: Obj, tau: Double, state: T): T {
        return state
    }

    fun range() = Range(tauStart, tauEnd)

    override fun toString() = "${name}:${range()}"
}

/**
 * Action just serving to mark a point at a proper time.
 */
open class Marker(tau: Double) : Action<Unit>(tau, tau) {
    override fun init() {}
}

/**
 * Creates a collision event if this object is at the same position as another (tracked) object.
 * Currently does *not* reduce time step if objects get close to another, but only detects collision at existing
 * events/simulation times.
 *
 * Only detects collisions during the specified interval (can be open-ended).
 */
open class DetectCollision(tau: Double, until: Double = Double.POSITIVE_INFINITY, vararg val targets: Obj) :
    Action<DetectCollision.MyState>(tau, until) {

    class MyState(val generated: Set<Obj> = setOf<Obj>())

    override fun init() = MyState()


    override fun act(world: WorldView, obj: Obj, tau: Double, state: MyState): MyState {
        val sourcePos = world.stateInFrame(obj)
        val added = mutableSetOf<Obj>()
        val removed = mutableSetOf<Obj>()
        for (target in targets) {
            val targetPos = world.stateInFrame(target)
            val dr = (targetPos.r.to3() - sourcePos.r.to3()).abs()
            // TODO if dr is small compared to time step reduce time step
            if (target in state.generated) {
                if (dr > eps * 2) removed.add(target)
            } else
                if (dr < eps * 2) {
                    collide(world, obj, sourcePos, target, targetPos)
                    added.add(target)
                }
        }
        return MyState(state.generated + added - removed)
    }

    /**
     * Records a collision by generating an Event (overridable) in
     * @param world view in which
     * @param self at
     * @param selfPos collides with (i.e. is close enough to)
     * @param targetObj at
     * @param targetPos in reference frame of the world
     */
    open fun collide(world: WorldView, self: Obj, selfPos: State, target: Obj, targetPos: State) {
        world.addEvent(Event("collide", this, selfPos.r, self, selfPos.tau, target, targetPos.tau))
    }
}

/**
 * A Sender action creates periodical [Pulse]s originating from its objects 4-vector.
 * The period is determined from object proper time.
 * @param name of the pulse (plus running number)
 * @param start of the sending
 * @param period of the pulses
 * @param no initial number of the pulse
 */
class Sender(override val name: String, val start: Double, val period: Double, val no: Int = 0) :
    Action<Unit>(start, start) {
    override fun init() {
    }

    override fun act(world: WorldView, obj: Obj, tau: Double, t: Unit) {
        if (abs(tau - start) < eps) {
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
open class Pulse(override val name: String, start: Double) : Action<Pulse.MyState>(start, Double.POSITIVE_INFINITY) {
    class MyState(
        val impossible: Set<Obj> = setOf<Obj>(),
        val tracked: Set<Obj> = setOf<Obj>(),
        val sourcePos: State? = null
    )

    override fun init() = MyState()

    override fun act(world: WorldView, obj: Obj, tau: Double, state: MyState): MyState {
        // note: sourcePos.t is transformed start tau
        // and objPos.t is transformed now tau
        val sourcePos = state.sourcePos ?: world.stateInFrame(obj).let { assert(it.tau == tau); it }

        val impossible = state.impossible.toMutableSet()
        val tracked = state.tracked.toMutableSet()

        for (newObj in world.objects - impossible - tracked) {
            val newObjPos = world.stateInFrame(newObj)
            when (separation(newObjPos.r, sourcePos.r, eps)) {
                Separation.TIMELIKE -> impossible.add(newObj)
                Separation.LIGHTLIKE -> {// immediate hit
                    strike(world, obj, sourcePos, newObj, newObjPos)
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
                    strike(world, obj, sourcePos, other, otherPos)
                    tracked.remove(other)
                    impossible.add(other)
                }
                Separation.SPACELIKE -> Unit // wait
            }
        }
        return MyState(impossible, tracked, sourcePos)
    }

    /**
     * Records the light pulse striking by generating an Event (overidable) in
     * @param world view in which
     * @param sourceObj at
     * @param sourcePos sent the light to
     * @param receiverObj at
     * @param receiverObjPos in reference frame of the world
     */
    open fun strike(world: WorldView, sourceObj: Obj, sourcePos: State, receiverObj: Obj, receiverObjPos: State) {
        world.addEvent(
            Event(
                name,
                this,
                receiverObjPos.r,
                sourceObj,
                sourcePos.tau,
                receiverObj,
                receiverObjPos.tau
            )
        )
    }
}
