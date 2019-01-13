package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.Vector3
import de.zarncke.timewarp.math.Vector4
import java.lang.IllegalArgumentException
import java.util.*
import kotlin.math.*

/**
 * Relativistic simulation.
 *
 * All computations with velocities (v) use units c=1.
 */
class TimeWarp {


    class World(
        var now: Double = 0.0,
        val objects: MutableList<Obj> = mutableListOf(),
        val completeActions: MutableSet<Action> = mutableSetOf(),
        val activeActions: MutableMap<Action, Obj> = mutableMapOf(),
        val events: MutableList<Event> = mutableListOf(),
        val states: MutableMap<Obj, State> = mutableMapOf()
    ) {
        var origin = Frame.ORIGIN
        var logActions = true
        var logMotions = true

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

        fun set(obj: Obj, state: State) {
            states[obj] = state
        }

        fun copy() = World(
            now,
            objects.toMutableList(),
            completeActions.toMutableSet(),
            activeActions.toMutableMap(),
            events.toMutableList(),
            states.toMutableMap()
        )
    }

    var world = World()
    val formula = Formula()

    fun init() {
    }

    fun addObj(obj: Obj, position: Vector3, velocity: Vector3 = V3_0) {
        world.addObj(obj)
        world.set(obj, State(position.to4(world.now), velocity, 0.0))
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
                    obj.actions().filter { !world.completeActions.contains(it) && !world.activeActions.keys.contains(it) }.firstOrNull()
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
                        val mcrf = state.toMCRF()
                        state = Inertial(state.tau, tauNext)
                            .moveUntilProperTime(mcrf, state.tau, tauNext)
                            .transform(mcrf, world.origin)
                    }

                    val mcrf = state.toMCRF()
                    val tauStart = max(tauNext, state.tau)   // not sure this matches with state/frame
                    val tauEnd = min(entry.value.tauEnd, tauAction)
                    // calculate 4-vector (in world frame) of the object at tau
                    state = entry.value.moveUntilProperTime(mcrf, tauStart, tauEnd).transform(mcrf, world.origin)
                }
                // (we may need to create an inertial movement afterwards)
                if (state.tau < tauAction) {
                    val mcrf = state.toMCRF()
                    state = Inertial(state.tau, tauAction)
                        .moveUntilProperTime(mcrf, state.tau, tauAction)
                        .transform(mcrf, world.origin)
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
                    val state = executeMotionToCoordinateTime(world.origin, world.states[obj]!!, obj, t)
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
                    val state = executeMotionToCoordinateTime(world.origin, world.states[obj]!!, obj, evaluatedTime)

                    worldNext.set(obj, state)
                }

                val changes = Action.Changes()
                for ((action, obj) in world.activeActions + (earliestAction to earliestObj!!)) {

                    // execute action a on o with the world and tau_o as parameter (may update events)
                    // execute all other active actions
                    try {
                        if (action == earliestAction)
                            earliestAction.act(worldNext, earliestObj, earliestState.tau, changes)
                        else {
                            val objState = worldNext.states[obj]!!
                            action.act(worldNext, obj, objState.tau, changes)
                        }
                    } catch (e: Action.RetrySmallerStep) {
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
            if (earliestAction.tauEnd != earliestAction.tauStart) {
                world.activeActions.put(earliestAction, earliestObj!!)
                // add an action that a) causes an call to act() at the end, b) completes the action in the world
                earliestObj.addAction(object : Action(earliestAction.tauEnd, earliestAction.tauEnd) {
                    override fun act(world: World, obj: Obj, tau: Double, changes: Changes) {
                        world.completeActions.add(earliestAction)
                        world.activeActions.remove(earliestAction)
                        if (world.logActions)
                            world.events.add(
                                Event(
                                    "Action-end:$earliestAction",
                                    earliestState.r,
                                    earliestObj!!,
                                    earliestObj
                                )
                            )
                    }
                })
            } else
                world.completeActions.add(earliestAction)

            if (world.logActions)
                world.events.add(Event("Action:$earliestAction", earliestState.r, earliestObj!!, earliestObj))
        }

        return world
    }

    /**
     * Helper to advance (evaluate its motion) an object to a given <em>coordinate time</em>.
     */
    private fun executeMotionToCoordinateTime(
        worldFrame: Frame,
        objState: State,
        obj: Obj,
        evaluatedTime: Double
    ): State {
        // execute motion of obj to target time
        //   set p = current position of obj
        //   set t = now
        //   while t<target time
        //     take next motion m (at current time t or later)
        //     if m is not at current time create inertial up the next one and execute that one
        //     calculate position q and tau for t with current motion m
        //     if tau > end of motion m
        //       calculate position p and t for end of motion
        //     else
        //       return q
        var state = objState
        assert(state.r.t == world.now)
        var objectCoordinateTime = state.r.t
        val motions = getMotionsInRange(obj, state.tau, Double.POSITIVE_INFINITY).entries.toMutableList()
        while (objectCoordinateTime < evaluatedTime && !motions.isEmpty()) {
            val entry = motions.removeAt(0)
            val tauNext = entry.key
            // (create ad-hoc inertial motion if needed)
            if (tauNext > state.tau) {
                state = Inertial(state.tau, tauNext).moveUntilCoordinateTime(state.toMCRF(), evaluatedTime)
            }

            if (tauNext < state.tau) {
                // (we need to undo motion back to its origin from where we can apply observer time)
                val s = state.toMCRF()
                state = entry.value.moveUntilProperTime(s, state.tau, entry.key)
                    .transform(s, worldFrame)
            } else {
                if (world.logMotions)
                    world.events.add(Event("Motion:${entry.value}", state.r, obj, obj))
            }
            state = entry.value.moveUntilCoordinateTime(state.toMCRF(), evaluatedTime)
            if (world.logMotions && state.tau == entry.value.tauEnd && entry.value.tauEnd != entry.value.tauStart)
                world.events.add(Event("Motion-end:${entry.value}", state.r, obj, obj))

            objectCoordinateTime = state.r.t
        }
        // (we may need to create an inertial movement afterwards)
        if (objectCoordinateTime < evaluatedTime) {
            state = Inertial(state.tau, Double.POSITIVE_INFINITY).moveUntilCoordinateTime(state.toMCRF(), evaluatedTime)
        }

        assert(abs(state.r.t - evaluatedTime) < eps, { "${state.r.t}!=$evaluatedTime" })
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


    data class Event(
        val name: String,
        val position: Vector4,
        val sender: Obj,
        val receiver: Obj
    )

}

// TODO move into context object?
var eps = 0.000001

/* Idea:
 * use Koma: mat[1.0,1.0,1.0].T*mat[1.0,1.0,1.0]
 */