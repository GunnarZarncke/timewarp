package de.zarncke.timewarp

import java.lang.IllegalArgumentException
import java.lang.IllegalStateException
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
        val completeActions: MutableSet<Action> = mutableSetOf(),
        val activeActions: MutableSet<Action> = mutableSetOf(),
        val events: MutableList<Event> = mutableListOf(),
        val states: MutableMap<Obj, State> = mutableMapOf()
    ) {
        var origin = Frame.ORIGIN

        fun addObj(obj: Obj) {
            objects.add(obj)
        }

        fun comovingFrame(obj: Obj) =
            (states[obj] ?: throw IllegalArgumentException("unknown object $obj"))
                .let { Frame(it.r, it.v) }

        /**
         * @param obj to get position and velocity for (at current time t in world frame)
         * @param s frame to get state in
         * @return state (position and velocity) in the given frame
         */
        fun stateInFrame(obj: Obj, s: Frame) =
            (states[obj] ?: throw IllegalArgumentException("unknown object $obj")).transform(origin, s)

        fun set(obj: TimeWarp.Obj, state: State) {
            states[obj] = state
        }

        fun copy() = World(
            now,
            objects.toMutableList(),
            completeActions.toMutableSet(),
            activeActions.toMutableSet(),
            events.toMutableList(),
            states.toMutableMap()
        )
    }

    var world = World()
    val formula = Formula()

    fun init() {
    }

    fun addObj(obj: Obj, position: Vector4) {
        world.addObj(obj)
        world.set(obj, State(position, V3_0, 0.0))
    }

    /**
     * Simulate world up to time t.
     * Determines all intermediate events of objects and calls [Obj.act] with a world updated to that time.
     * @param t coordinate time in world frame ('origin')
     * @return World at coordinate time t
     */
    fun simulateTo(t: Double): World {
        // note: processing from any observer frame should lead to the same results
        // while simulation target time not yet reached
        while (world.now < t) {
            // for each object
            //   among unhandled actions not earlier than now
            //     determine proper time tau of first scheduled action
            //     determine all scheduled motions up to that event (might require creation of ad-hoc inertial moves)
            //     simulate all motions up to tau
            //     calculate 4-vector (in world frame) of the object at tau
            // take the earliest of these 4-vectors call it r and its object o and action a
            var earliestState: State? = null
            var earliestAction: Action? = null
            var earliestObj: Obj? = null
            for (obj in world.objects) {
                // among unhandled actions not earlier than now
                val nextAction =
                    obj.actions().filter { !world.completeActions.contains(it) && !world.activeActions.contains(it) }.firstOrNull()
                        ?: continue
                //  determine proper time tau of first scheduled action
                var state = world.states[obj]!!
                val tauAction = nextAction.tauStart

                // determine all scheduled motions up to that event (might require creation of ad-hoc inertial moves)
                val motions = getMotionsInRange(obj, state.tau, tauAction)

                // simulate all motions up to tau
                for (entry in motions) {
                    val tauNext = entry.key
                    // (create ad-hoc inertial motion if needed)
                    if (tauNext > state.tau) {
                        val s = Frame(state.r, state.v)
                        state = Inertial(state.tau, tauNext)
                            .moveUntilProperTime(obj, s, state.tau, tauNext).transform(s, world.origin)
                    }

                    val s = Frame(state.r, state.v)
                    val tauStart = max(tauNext, state.tau)   // not sure this matches with state/frame
                    val tauEnd = min(entry.value.tauEnd, tauAction)
                    // calculate 4-vector (in world frame) of the object at tau
                    state = entry.value.moveUntilProperTime(obj, s, tauStart, tauEnd).transform(s, world.origin)
                }
                // (we may need to create an inertial movement afterwards)
                if (state.tau < tauAction) {
                    val s = Frame(state.r, state.v)
                    state = Inertial(state.tau, tauAction)
                        .moveUntilProperTime(obj, s, state.tau, tauAction).transform(s, world.origin)
                }
                // take the earliest of these 4-vectors call it r and its object o and action a
                if (earliestState == null || earliestState.r.t < state.r.t) {
                    earliestState = state
                    earliestAction = nextAction
                    earliestObj = obj
                }
            }

            if (earliestAction == null) {
                // (no further actions; we continue motion directly to the end state)
                for (obj in world.objects) {
                    val state = executeMotionToCoordinateTime(world, obj, t)
                    world.set(obj, state)
                }
                world.now = t
                break;
            }

            // extra handling for reduced time steps
            val targetTime = earliestState!!.r.t
            var fallbackTime = world.now
            var evaluatedTime = targetTime
            time@
            while (true) {
                // create a candidate world - we might need to backtrack
                val worldNext = world.copy()

                // for each object
                //   execute motion to r (see below) determining tau_o
                //   update the object in the world to it's 4-vector
                for (obj in worldNext.objects) {
                    if (obj == earliestObj) {
                        // for the earliest action we know the state and take a shortcut
                        worldNext.set(obj, earliestState)
                        continue
                    }
                    val state = executeMotionToCoordinateTime(worldNext, obj, evaluatedTime)

                    worldNext.set(obj, state)
                }

                val changes = Action.Changes()
                for (action in world.activeActions + earliestAction) {

                    // execute action a on o with the world and tau_o as parameter (may update events)
                    // execute all other active actions
                    try {
                        earliestAction.act(worldNext, earliestObj!!, earliestState.tau, changes)
                    } catch (e: RetrySmallerStep) {
                        // stop, we went too far ahead
                        evaluatedTime = (fallbackTime + evaluatedTime) / 2
                        continue@time
                    }
                }

                // (we successfully executed all actions for the evaluated time and can update the world to it)
                changes.applyChanges(worldNext)
                world = worldNext
                if (evaluatedTime == targetTime)
                    break
                fallbackTime = evaluatedTime
                evaluatedTime = targetTime
            }

            // mark action a as done
            if (earliestAction.tauEnd != earliestAction.tauStart)
                world.activeActions.add(earliestAction)
            else world.completeActions.add(earliestAction)

            //world.events.add(Event("Action:$action", pos, obj, obj))
        }

        return world
    }

    private fun executeMotionToCoordinateTime(world: World, obj: Obj, evaluatedTime: Double): State {
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
        var state = world.states[obj]!!
        var objectCoordinateTime = world.now
        val motions = getMotionsInRange(obj, state.tau, Double.POSITIVE_INFINITY).entries.toMutableList()
        while (objectCoordinateTime < evaluatedTime && !motions.isEmpty()) {
            val entry = motions.removeAt(0)
            val tauNext = entry.key
            // (create ad-hoc inertial motion if needed)
            if (tauNext > state.tau) {
                val s = Frame(state.r, state.v)
                val transformedCoordinateTime =
                    state.copy(r = state.r.copy(t = evaluatedTime)).transform(world.origin, s).r.t
                state = Inertial(state.tau, tauNext)
                    .moveUntilObserverTime(obj, s, transformedCoordinateTime)
                    .transform(s, world.origin)
            }

            if (tauNext < state.tau) {
                // (we need to undo motion back to its origin from where we can apply observer time)
                val s = Frame(state.r, state.v)
                state = entry.value.moveUntilProperTime(obj, s, state.tau, entry.key)
                    .transform(s, world.origin)
            }
            val s = Frame(state.r, state.v)
            // TODO much unused transform included here, might simplify:
            val transformedCoordinateTime =
                state.copy(r = state.r.copy(t = evaluatedTime)).transform(world.origin, s).r.t
            state = entry.value.moveUntilObserverTime(obj, s, transformedCoordinateTime)
                .transform(s, world.origin)
            objectCoordinateTime = state.r.t
        }
        // (we may need to create an inertial movement afterwards)
        if (objectCoordinateTime < evaluatedTime) {
            val s = Frame(state.r, state.v)
            val transformedCoordinateTime =
                state.copy(r = state.r.copy(t = evaluatedTime)).transform(world.origin, s).r.t
            state = Inertial(state.tau, Double.POSITIVE_INFINITY)
                .moveUntilObserverTime(obj, s, transformedCoordinateTime)
                .transform(s, world.origin)
        }

        if (state.r.t != evaluatedTime)
            throw IllegalStateException()
        return state
    }

    private fun getMotionsInRange(obj: Obj, tauStart: Double, tauEnd: Double): SortedMap<Double, Motion> {
        val motions = obj.motions().subMap(tauStart, tauEnd).toSortedMap()
        // (there may be an incomplete motion started earlier)
        val prev = obj.motions().headMap(tauStart).entries.lastOrNull()
        if (prev != null && prev.value.tauStart < tauStart && prev.value.tauEnd > tauStart) {
            motions.put(prev.key, prev.value)
        }
        return motions
    }

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
            val overlaps = motions.subMap(motion.tauStart, motion.tauEnd)
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

    class RetrySmallerStep : Exception()

    abstract class Motion(val tauStart: Double, val tauEnd: Double) {

        /**
         * Determines the location and velocity of an object at a given <em>proper time</em> tauTo within a co-moving reference frame.
         * This method will be called when the proper time of the associated object is greater or equal the motion start time.
         * It may be called multiple times with different values (or even the same value) for tauNow and tauTo (in corresponding reference frames).
         * @param obj in
         * @param coMovingFrame co-moving with object at
         * @param tauNow proper time of object (sometime during the motion) to
         * @param tauTo proper time of object (may be *before* tauNow, but not before tauStart)
         * @return state (4-vector and velocity of object at proper time tau within given frame)
         */
        abstract fun moveUntilProperTime(obj: Obj, coMovingFrame: Frame, tauNow: Double, tauTo: Double): State

        // TODO check what happens with lateral acceleration, maybe this coMovingFrame thing is not yet well thought out

        /**
         * Determines the location and velocity of an object at a given <em>coordinate time</em> t within the given reference frame.
         * This method will be called to determine the location and proper time of the associated object at coordinate time.
         * @param obj at
         * @param t coordinate time of object in
         * @param coMovingFrame co-moving with object at time tauStart (!)
         * @return state (4-vector and velocity and tau of object either at t or the end of the motion whatever is earlier)
         */
        abstract fun moveUntilObserverTime(obj: Obj, coMovingFrame: Frame, t: Double): State
    }

    /**
     * Inertial motion stays at the origin of its comoving reference frame.
     * If a velocity is specified it applies at tauStart.
     */
    class Inertial(tauStart: Double, tauEnd: Double, val v: Vector3 = V3_0) : Motion(tauStart, tauEnd) {
        override fun moveUntilProperTime(obj: Obj, coMovingFrame: Frame, tauNow: Double, tauTo: Double): State {
            return State((v * (tauTo - tauNow)).to4(tauTo - tauNow), v, tauNow)
        }

        override fun moveUntilObserverTime(obj: Obj, coMovingFrame: Frame, t: Double): State {
            // t = tau for inertial motion
            // see https://en.wikipedia.org/wiki/Proper_time
            var dt = t - coMovingFrame.r.t
            if (this.tauStart + dt > tauEnd) dt = tauEnd - tauStart
            val p = (v * dt).to4(dt)
            return State(p, v, this.tauStart + dt)
        }
    }

    /**
     * "a) Hyperbolic motion: The constant, longitudinal proper acceleration
     * {\alpha =a_{x}^{0}=a_{x}\gamma ^{3}} by (4a) leads to the world line..."
     * https://en.wikipedia.org/wiki/Acceleration_(special_relativity)#Curved_world_lines
     */
    class LongitudinalAcceleration(tauStart: Double, tauEnd: Double, val a: Vector3) : Motion(tauStart, tauEnd) {
        override fun moveUntilProperTime(obj: Obj, coMovingFrame: Frame, tauNow: Double, tauTo: Double): State {
            return relativisticAcceleration(a, tauTo - tauNow).copy(tau = tauTo)
        }

        override fun moveUntilObserverTime(
            obj: Obj,
            coMovingFrame: Frame,
            t: Double
        ): State {
            val state = relativisticCoordAcceleration(a, t)

            if (this.tauStart + state.tau < tauEnd)
                return state.copy(tau = state.tau + tauStart)
            return relativisticAcceleration(a, tauEnd - tauStart).copy(tau = tauEnd)
        }
    }

    abstract class Action(val tauStart: Double, val tauEnd: Double) {
        data class Changes(
            val actions: MutableSet<Pair<Obj, Action>> = mutableSetOf(),
            val motions: MutableList<Pair<Obj, Motion>> = mutableListOf(),
            val objects: MutableSet<Pair<Obj, Vector4>> = mutableSetOf(),
            val events: MutableSet<Event> = mutableSetOf()
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
        open fun act(world: World, obj: Obj, tau: Double, changes: Changes) {}

        /**
         *
         */
        open fun always() = false
    }

    /**
     * Creates a collision event if this object is at the same position as another (tracked) object.
     * Currently does *not* reduce time step if objects get close to another, but only detects collision at existing
     * events/simulation times.
     */
    open class DetectCollision(tau: Double, val until: Double, val targets: Set<Obj>) :
        Action(tau, Double.POSITIVE_INFINITY) {
        private val generated = mutableSetOf<Obj>()
        override fun act(world: World, obj: Obj, tau: Double, changes: Changes) {
            val sourcePos = world.stateInFrame(obj, world.origin)
            for (target in targets - generated) {
                val objPos = world.stateInFrame(target, world.origin)
                val dr = (objPos.r.to3() - sourcePos.r.to3()).abs()
                // TODO if dr is small compared to time step reduce time step
                if (dr < eps) {
                    collide(changes, objPos, obj, target)
                    generated.add(target)
                } else if (dr > eps) generated.remove(target)
            }
        }

        open fun collide(changes: Changes, objPos: State, self: Obj, obj: Obj) {
            changes.events.add(Event("collide", objPos.r, self, obj))
        }
    }

    /**
     * A Sender action creates periodical [Pulse]s originating from its objects 4-vector.
     * The period is determined from object proper time.
     */
    class Sender(val name: String, val start: Double, val period: Double, val no: Int = 0) : Action(start, start) {
        override fun act(world: World, obj: Obj, tau: Double, changes: Changes) {
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
        private val generated = mutableSetOf<Obj>()
        override fun act(world: World, obj: Obj, tau: Double, changes: Changes) {
            // note: sourcePos.t is transformed start tau
            // and objPos.t is transformed now tau
            val sourcePos = world.stateInFrame(obj, world.origin).r
            for (other in world.objects - generated) {
                val objPos = world.stateInFrame(other, world.origin).r
                val dr = (objPos.to3() - sourcePos.to3()).abs()
                val dt = objPos.t - sourcePos.t
                if (abs(dr - dt) < eps) {
                    changes.events.add(Event(name, objPos, obj, other))
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

/**
 * A frame is a coordinate system that is relative to another coordinate system (by default the origin).
 * Note: No rotation supported yet.
 * @property r relative position to other frame as measured by the other frame
 * @property v relative velocity as measured by the other  frame
 */
class Frame(val r: Vector4, val v: Vector3) {
    companion object {
        val ORIGIN = Frame(V4_0, V3_0)
    }

    fun isOrigin() = r == V4_0 && v == V3_0
}

/**
 * State of an object in a frame (by default world origin frame).
 * @property r position in corresponding frame
 * @property v velocity in corresponding frame
 * @property tau local proper time object
 */
data class State(val r: Vector4, val v: Vector3, val tau: Double) {
    fun transform(from: Frame, to: Frame): State {
        val s = if (from.isOrigin()) this else
            State(lorentzTransformInv(from.v, r) + from.r, observedAddedVelocity(from.v, v), tau)
        if (to.isOrigin()) return s
        return State(lorentzTransform(to.v, s.r - to.r), transformedAddedVelocity(to.v, s.v), tau)
    }
}
