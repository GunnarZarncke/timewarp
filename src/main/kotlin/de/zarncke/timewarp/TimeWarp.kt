package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.Vector3
import koma.min
import java.util.*
import java.util.logging.Logger
import kotlin.math.abs
import kotlin.math.max

/**
 * Relativistic simulation.
 *
 * All computations with velocities (v) use units c=1.
 */
class TimeWarp(private val logger: Logger = Logger.getLogger(TimeWarp::javaClass.name)) {
    /*
     * Ideas:
     * - simulate up to proper time tau of an object
     * - simulate up to the next/a specific event
     */
    interface Observer {
        /**
         * Called when the simulation has committed to simulated time step.
         * @param world at that time
         * @return true: abort simulation
         */
        fun observe(world: WorldView): Boolean
    }

    private val observers = mutableSetOf<Observer>()
    fun addObserver(observer: Observer) {
        observers.add(observer)
    }

    fun removeObserver(observer: Observer) {
        observers.remove(observer)
    }

    private var world = World()
    val theWorld: WorldView
        get() = object : WorldView {
            override val origin: Frame get() = world.origin
            override val objects: Collection<Obj> get() = world.objects
            override val logActions: Boolean get() = world.logActions
            override val events: List<Event>
                get() = world.events.sortedWith(
                    compareBy(
                        Event::position,
                        Event::name,
                        Event::cause
                    )
                )

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

            override fun complete(action: Action<*>) {
                throw UnsupportedOperationException("internal use only")
            }

            override fun deactivate(action: Action<*>) {
                throw UnsupportedOperationException("internal use only")
            }

            override val activeActions: Map<Action<*>, Obj>
                get() = throw UnsupportedOperationException("internal use only")

            override val completeActions: Set<Action<*>>
                get() = throw UnsupportedOperationException("internal use only")

            override fun addActiveAction(action: Action<*>, obj: Obj) =
                throw UnsupportedOperationException("internal use only")

            override fun stateInFrame(obj: Obj, frame: Frame) = world.stateInFrame(obj, frame)
            override fun actionState(action: Action<*>): Any {
                throw UnsupportedOperationException("internal use only")
            }

            override fun setAState(action: Action<*>, state: Any?) {
                throw UnsupportedOperationException("internal use only")
            }
        }

    fun addObj(obj: Obj, r: Vector3, v: Vector3 = V3_0, tau: Double = 0.0) {
        world.addObj(obj, r.to4(world.now), v, tau)
    }

    fun events(
        place: Vector3? = null,
        time: Double? = null,
        name: String? = null,
        nameRegex: Regex? = null,
        tau: Double? = null,
        receiver: Obj? = null,
        sender: Obj? = null,
        causeClass: Class<*>? = null
    ) = theWorld.events.filter {
        if (place != null && place != it.position.to3()) false
        else if (time != null && time != it.position.t) false
        else if (name != null && name != it.name) false
        else if (nameRegex != null && !it.name.matches(nameRegex)) false
        else if (sender != null && sender != it.sender) false
        else if (receiver != null && receiver != it.receiver) false
        else if (tau != null && abs(tau - it.tauReceiver) > eps) false
        else if (causeClass != null && !causeClass.isInstance(it.cause)) false
        else true
    }

    data class Activity(
        var state: State,
        var action: Action<*>,
        var obj: Obj
    )


    /**
     * Simulate world up to time t.
     * Determines all intermediate events of objects and calls [Action.act] with a world updated to that time.
     * @param t coordinate time in world frame ('origin')
     * @return World snapshot at coordinate time t
     */
    fun simulateTo(t: Double): World {
        class Finisher(private val activity: Activity) : Action<Unit>(activity.action.tauEnd, activity.action.tauEnd) {
            override fun init() {}
            override val isSilent: Boolean get() = true

            override fun act(world: WorldView, obj: Obj, tau: Double, state: Unit) {
                world.complete(activity.action)
                if (world.logActions && !activity.action.isSilent)
                    world.addEvent(
                        Event(
                            "Action-end",
                            activity.action,
                            activity.state.r,
                            activity.obj,
                            activity.state.tau,
                            obj,
                            tau
                        )
                    )
            }

            override fun toString() = "Finish($tauEnd)"
        }

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
                    if (tauStart == tauEnd) {
                        // skip
                        logger.fine { "skipping non-move at $tauStart" }
                    } else
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
                // we know that tau must be the analytic proper time
                state = state.exactTau(tauAction)
                // take the earliest of these 4-vectors call it r and its object o and action a
                if (earliest == null || state.r.t < earliest.state.r.t) {
                    earliest = Activity(state, nextAction, obj)
                }
            }

            if (earliest != null && earliest.state.r.t > t) {
                earliest = null
            }
            // now we have determined the earliest applicable action of an object and its state, but no change yet

            // no action is in range -> quick exit
            if (world.activeActions.isEmpty() && earliest == null) {
                // (no further actions; we continue motion directly to the end state and update world directly)
                for (obj in world.objects) {
                    val state = executeMotionToCoordinateTime(world.origin, world.stateInFrame(obj), obj, t)
                    if (state.r.t != t)
                        assert(false)
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
                logger.info("evaluate at $evaluatedTime")
                // create a candidate world - we might need to backtrack
                val space = Space()

                // for each object
                //   execute motion to r (see below) determining tau_o
                //   update the object in the world to it's 4-vector
                for (obj in world.objects) {
                    val state =
                        if (obj == earliest?.obj && earliest.state.r.t == evaluatedTime) earliest.state // if we already have this state exactly we use it
                        else executeMotionToCoordinateTime(world.origin, world.stateInFrame(obj), obj, evaluatedTime)
                    if (state.r.t != evaluatedTime)
                        assert(false)
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
                            (action as Action<Any?>).act(worldNext, obj, objState.tau, worldNext.actionState(action))
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
                logger.info("commit to $evaluatedTime")
                world = worldNext.applyAll()

                if (evaluatedTime >= targetTime)
                    break
                fallbackTime = evaluatedTime
                evaluatedTime = targetTime
            }

            // mark action a as done (if it was actually reached and not some other action cut it short)
            if (earliest != null && evaluatedTime == earliest.state.r.t) {
                if (earliest.action.tauEnd != earliest.action.tauStart) {
                    world.addActiveAction(earliest.action, earliest.obj)
                    // add an action that a) causes a call to act() at the end, b) completes the action in the world
                    if (earliest.action.tauEnd != Double.POSITIVE_INFINITY)
                        earliest.obj.addAction(Finisher(earliest))
                } else
                    world.complete(earliest.action)

                if (world.logActions && !earliest.action.isSilent)
                    world.addEvent(
                        Event(
                            "Action",
                            earliest.action,
                            earliest.state.r,
                            earliest.obj,
                            earliest.state.tau,
                            earliest.obj,
                            earliest.state.tau
                        )
                    )
            }
            val anyStop = observers.map { it.observe(theWorld) }.any()
            if (anyStop) {
                logger.info("Stop by observer(s)")
                break
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
                if (world.logMotions && !entry.value.isSilent)
                    world.addEvent(
                        Event(
                            "Motion",
                            entry.value,
                            state.r,
                            obj,
                            state.tau,
                            obj,
                            state.tau
                        )
                    )
            }
            state = entry.value.moveUntilCoordinateTime(state.toMCRF(), evaluatedTime)
            if (world.logMotions && !entry.value.isSilent && state.tau == entry.value.tauEnd && entry.value.tauEnd != entry.value.tauStart)
                world.addEvent(
                    Event(
                        "Motion-end",
                        entry.value,
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

        assert(abs(state.r.t - evaluatedTime) < eps)
        { "${state.r.t}!=$evaluatedTime" }
        // analytically the time component is exact and we set it thus
        return state.copy(r = state.r.copy(t = evaluatedTime))
    }

    private fun getMotionsInRange(obj: Obj, tauStart: Double, tauEnd: Double): SortedMap<Double, Motion> {
        if (tauStart > tauEnd)
            throw IllegalStateException()
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
val eps = 0.00000001
val precision = eps

/* Idea:
 * use Koma: mat[1.0,1.0,1.0].T*mat[1.0,1.0,1.0]
 */