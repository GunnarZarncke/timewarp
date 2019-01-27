package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.Vector3
import java.lang.IllegalArgumentException
import java.util.*
import java.util.logging.Logger
import kotlin.math.*

/**
 * Relativistic simulation.
 *
 * All computations with velocities (v) use units c=1.
 */
class TimeWarp(private val logger: Logger = Logger.getLogger(TimeWarp::javaClass.name)) {

    var world = World()
    val formula = Formula()

    fun init() {
    }

    fun addObj(obj: Obj, r: Vector3, v: Vector3 = V3_0, tau: Double = 0.0) {
        world.addObj(obj, r, v, tau)
    }

    /*
     * Ideas:
     * - simulate up to proper time tau of an object
     * - simulate up to the next/a specific event
     */

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
                    obj.actions().first { !world.completeActions.contains(it) && !world.activeActions.keys.contains(it) }
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

            // now we have determined the earliest applicable action of an object and its state, but no change yet

            if (earliestAction == null) {
                // (no further actions; we continue motion directly to the end state)
                for (obj in world.objects) {
                    val state = executeMotionToCoordinateTime(world.origin, world.states[obj]!!, obj, t)
                    world.set(obj, state)
                }
                world.now = t
                break;
            }


            // note: loop is extra handling for reduced time steps
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
                    val state =
                        executeMotionToCoordinateTime(worldNext.origin, worldNext.states[obj]!!, obj, evaluatedTime)

                    worldNext.set(obj, state)
                }

                worldNext.now = evaluatedTime
                // now we have initialized the candidate future world to the time of a potential or real event

                // execute action a on o with the world and tau_o as parameter (may update events)
                // and execute all other active actions
                val changes = Changes()
                for ((action, obj) in worldNext.activeActions + (earliestAction to earliestObj!!)) {

                    try {
                        if (action == earliestAction)
                            earliestAction.act(worldNext, earliestObj, earliestState.tau, changes)
                        else {
                            val objState = worldNext.states[obj]!!
                            action.act(worldNext, obj, objState.tau, changes)
                        }
                    } catch (e: Action.RetrySmallerStep) {
                        // stop, we went too far ahead
                        if (Math.abs(fallbackTime - evaluatedTime) < precision) {
                            logger.warning("too high precision requirement for $earliestAction ($fallbackTime, $evaluatedTime)")
                        } else {
                            evaluatedTime = (fallbackTime + evaluatedTime) / 2
                            continue@time
                        }
                    }
                }

                // (we successfully executed all actions for the evaluated time and can update the world to it)
                changes.applyChanges(worldNext)

                world = worldNext
                if (evaluatedTime >= targetTime)
                    break
                fallbackTime = evaluatedTime
                evaluatedTime = targetTime
            }

            // mark action a as done
            if (earliestAction.tauEnd != earliestAction.tauStart) {
                world.activeActions.put(earliestAction, earliestObj!!)
                // add an action that a) causes a call to act() at the end, b) completes the action in the world
                earliestObj.addAction(object : Action(earliestAction.tauEnd, earliestAction.tauEnd) {
                    override fun act(world: WorldView, obj: Obj, tau: Double, changes: Changes) {
                        changes.completions.add(earliestAction)
                        if (world.logActions)
                            changes.events.add(
                                Event(
                                    "Action-end:$earliestAction",
                                    earliestState.r,
                                    earliestObj,
                                    earliestState.tau,
                                    obj,
                                    tau
                                )
                            )
                    }
                })
            } else
                world.completeActions.add(earliestAction)

            if (world.logActions)
                world.events.add(
                    Event(
                        "Action:$earliestAction",
                        earliestState.r,
                        earliestObj!!,
                        earliestState.tau,
                        earliestObj,
                        earliestState.tau
                    )
                )
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
        if (state.r.t != world.now)
            assert(false)
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
                    world.events.add(
                        Event(
                            "Motion:${entry.value}",
                            state.r,
                            obj,
                            state.tau,
                            obj,
                            state.tau
                        )
                    )
            }
            state = entry.value.moveUntilCoordinateTime(state.toMCRF(), evaluatedTime)
            if (world.logMotions && state.tau == entry.value.tauEnd && entry.value.tauEnd != entry.value.tauStart)
                world.events.add(
                    Event(
                        "Motion-end:${entry.value}",
                        state.r,
                        obj,
                        state.tau,
                        obj,
                        state.tau
                    )
                )

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


    override fun toString() = "simulate $world"

}

// TODO move into context object?
val eps = 0.000001
val precision = eps

/* Idea:
 * use Koma: mat[1.0,1.0,1.0].T*mat[1.0,1.0,1.0]
 */