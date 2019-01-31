package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.Vector3
import koma.min
import java.lang.IllegalStateException
import java.lang.UnsupportedOperationException
import java.util.*
import java.util.logging.Logger
import kotlin.Comparator
import kotlin.math.*

/**
 * Relativistic simulation.
 *
 * All computations with velocities (v) use units c=1.
 */
class TimeWarp(private val logger: Logger = Logger.getLogger(TimeWarp::javaClass.name)) {

    private var world = World()
    val theWorld: WorldView
        get() = object : WorldView {
            override val origin: Frame get() = world.origin
            override val objects: Collection<Obj> get() = world.objects
            override val logActions: Boolean get() = world.logActions
            override val events: List<Event> get() = world.events.sortedWith(compareBy(Event::position, Event::name))
            override fun addEvent(e: Event) {
                world.addEvent(e)
            }

            override fun <T> addAction(obj: Obj, action: Action<T>) {
                world.addAction(obj, action)
            }

            override fun addMotion(obj: Obj, motion: Motion) {
                world.addMotion(obj, motion)
            }

            override fun addOrSetObject(obj: Obj, state: State) {
                world.addOrSetObject(obj, state)
            }

            override fun complete(action: Action<Any>) {
                throw UnsupportedOperationException("internal user only")
            }

            override fun stateInFrame(obj: Obj, frame: Frame) = world.stateInFrame(obj, frame)
            override fun actionState(action: Action<Any>): Any {
                throw UnsupportedOperationException("internal user only")
            }

            override fun setAState(action: Action<Any>, state: Any) {
                throw UnsupportedOperationException("internal user only")
            }
        }

    fun addObj(obj: Obj, r: Vector3, v: Vector3 = V3_0, tau: Double = 0.0) {
        world.addObj(obj, r, v, tau)
    }

    /*
     * Ideas:
     * - simulate up to proper time tau of an object
     * - simulate up to the next/a specific event
     */

    data class Activity(
        var state: State,
        var action: Action<Any>,
        var obj: Obj
    )


    /**
     * Simulate world up to time t.
     * Determines all intermediate events of objects and calls [Action.act] with a world updated to that time.
     * @param t coordinate time in world frame ('origin')
     * @return World snapshot at coordinate time t
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
            var earliest: Activity? = null
            for (obj in world.objects) {
                // among unhandled actions not earlier than now
                val nextAction =
                    obj.actions().firstOrNull {
                        !world.completeActions.contains(it) && !world.activeActions.keys.contains(it)
                    } ?: continue
                //  determine proper time tau of first scheduled action
                var state = world.stateInFrame(obj)
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
                if (earliest == null || earliest.state.r.t < state.r.t) {
                    earliest = Activity(state, nextAction, obj)
                }
            }

            if (earliest != null && earliest.state.r.t > t) {
                earliest = null
            }
            // now we have determined the earliest applicable action of an object and its state, but no change yet

            // no action is in range -> quick exit (we test both vars to make the compiler happy)
            if (world.activeActions.isEmpty() && earliest == null) {
                // (no further actions; we continue motion directly to the end state and update world directly)
                for (obj in world.objects) {
                    val state = executeMotionToCoordinateTime(world.origin, world.stateInFrame(obj), obj, t)
                    world.space.set(obj, state)
                }
                world.now = t
                break
            }

            // note: loop is extra handling for reduced time steps
            var targetTime = if (earliest == null) t else earliest.state.r.t
            var fallbackTime = world.now
            var evaluatedTime = targetTime
            var numOfTries = 0

            time@ while (true) {
                // create a candidate world - we might need to backtrack
                val space = Space()

                // for each object
                //   execute motion to r (see below) determining tau_o
                //   update the object in the world to it's 4-vector
                for (obj in world.objects) {
                    val state = executeMotionToCoordinateTime(world.origin, world.stateInFrame(obj), obj, evaluatedTime)

                    space.set(obj, state)
                }

                val worldNext = DeltaWorld(world, space, evaluatedTime)
                // now we have initialized the candidate future world to the time of a potential or real event

                // execute action a on o with the world and tau_o as parameter (may update events)
                // and execute all other active actions
                val activeActions =
                    if (earliest == null) world.activeActions else world.activeActions + (earliest.action to earliest.obj)
                for ((action, obj) in activeActions) {
                    try {
                        // TODO we capture state changes of action but not which actions are added!!!
                        // TODO need to store assignment of actions to objects in world state
                        val objState = worldNext.stateInFrame(obj)
                        worldNext.setAState(
                            action,
                            action.act(worldNext, obj, objState.tau, worldNext.actionState(action))
                        )
                    } catch (e: Action.RetrySmallerStep) {
                        numOfTries++
                        if (numOfTries > 64) throw IllegalStateException("too many retries of $action")
                        // stop, we went too far ahead
                        if (Math.abs(fallbackTime - evaluatedTime) < precision) {
                            logger.warning("too high precision requirement for $earliest ($fallbackTime, $evaluatedTime)")
                        } else {
                            // if a hint is given we limit the hint to 10% to 90% of the interval
                            // if no hint is given we use 50% of the interval
                            val evaluatedTimeOld = evaluatedTime
                            evaluatedTime =
                                    if (e.tHint != null && e.tHint > fallbackTime && e.tHint < evaluatedTime) {
                                        val minStep = (evaluatedTime - fallbackTime) / 10
                                        min(max(e.tHint, fallbackTime + minStep), evaluatedTime - minStep)
                                    } else (fallbackTime + evaluatedTime) / 2
                            logger.info("retry by $action to $evaluatedTime in $fallbackTime...$targetTime")
                            targetTime = evaluatedTimeOld
                            continue@time
                        }
                    }
                }

                // (we successfully executed all actions for the evaluated time and can update the world to it)
                world = worldNext.applyAll()

                if (evaluatedTime >= targetTime)
                    break
                fallbackTime = evaluatedTime
                evaluatedTime = targetTime
            }

            // mark action a as done
            if (earliest != null) {
                if (earliest.action.tauEnd != earliest.action.tauStart) {
                    world.activeActions[earliest.action] = earliest.obj
                    // add an action that a) causes a call to act() at the end, b) completes the action in the world
                    earliest.obj.addAction(object : Action<Unit>(earliest.action.tauEnd, earliest.action.tauEnd) {
                        override fun init() {}

                        override fun act(world: WorldView, obj: Obj, tau: Double, t: Unit) {
                            world.complete(earliest.action)
                            if (world.logActions)
                                world.addEvent(
                                    Event(
                                        "Action-end:${earliest.action}",
                                        earliest.state.r,
                                        earliest.obj,
                                        earliest.state.tau,
                                        obj,
                                        tau
                                    )
                                )
                        }
                    })
                } else
                    world.completeActions.add(earliest.action)

                if (world.logActions)
                    world.events.add(
                        Event(
                            "Action:${earliest.action}",
                            earliest.state.r,
                            earliest.obj,
                            earliest.state.tau,
                            earliest.obj,
                            earliest.state.tau
                        )
                    )
            }
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

        assert(abs(state.r.t - evaluatedTime) < eps) { "${state.r.t}!=$evaluatedTime" }
        return state
    }

    private fun getMotionsInRange(obj: Obj, tauStart: Double, tauEnd: Double): SortedMap<Double, Motion> {
        val motions = obj.motions().subMap(tauStart, tauEnd).toSortedMap()
        // (there may be an incomplete motion started earlier)
        val prev = obj.motions().headMap(tauStart).entries.lastOrNull()
        if (prev != null && prev.value.tauStart < tauStart && prev.value.tauEnd > tauStart) {
            motions[prev.key] = prev.value
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