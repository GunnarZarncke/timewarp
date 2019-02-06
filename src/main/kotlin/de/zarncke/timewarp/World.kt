package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.V4_0
import de.zarncke.timewarp.math.Vector3
import de.zarncke.timewarp.math.Vector4
import java.lang.IllegalArgumentException

interface WorldView {
    fun addEvent(e: Event)
    fun <T> addAction(obj: Obj, action: Action<T>)
    fun addMotion(obj: Obj, motion: Motion)
    fun addOrSetObject(obj: Obj, state: State)
    fun complete(action: Action<*>)

    /**
     * @param obj to get position and velocity for (at current time t in world frame)
     * @param frame frame to get state in
     * @return state (position and velocity) in the given frame
     */
    fun stateInFrame(obj: Obj, frame: Frame = origin): State

    fun actionState(action: Action<*>): Any?
    fun setAState(action: Action<*>, state: Any?)
    fun deactivate(action: Action<*>)

    val origin: Frame
    val objects: Collection<Obj>
    val activeActions: Map<Action<*>, Obj>
    val completeActions: Set<Action<*>>
    val events: List<Event>
    val logActions: Boolean
    fun addActiveAction(action: Action<*>, obj: Obj)
}

class DeltaWorld(private val base: World, private val space: Space, val now: Double) : WorldView {
    private val changes = Changes()

    fun applyAll(): World {
        val newWorld = base.copyWith(space, now)
        changes.applyChanges(newWorld)
        return newWorld
    }

    override val origin: Frame get() = base.origin
    override val objects: Collection<Obj> get() = base.objects
    override val events: List<Event> get() = base.events
    override val logActions: Boolean get() = base.logActions
    override val activeActions: Map<Action<*>, Obj> = base.activeActions
    override fun addActiveAction(action: Action<*>, obj: Obj) = base.addActiveAction(action, obj)

    override val completeActions: Set<Action<*>> = base.completeActions

    override fun setAState(action: Action<*>, state: Any?) {
        changes.actionStates[action] = state
    }

    override fun actionState(action: Action<*>) = base.actionState(action)

    override fun addEvent(e: Event) {
        changes.events.add(e)
    }

    override fun <T> addAction(obj: Obj, action: Action<T>) {
        changes.actions.add(obj to action as Action<*>)
    }

    override fun addMotion(obj: Obj, motion: Motion) {
        changes.motions.add(obj to motion)
    }

    override fun addOrSetObject(obj: Obj, state: State) {
        if (state.r.t != now) {
            if (state.r.t < now) throw IllegalArgumentException(
                "Cannot add objects in the past ($state.r.t) of the simulation frame " +
                        "${origin.r.copy(t = now)}"
            )
            // for objects that appear in the future we create the object in
            // the simulation inertial frame now at the same space coordinates and
            // add an action to set it in motion once its time has come.
            obj.addMotion(object: AbruptVelocityChange(state.tau, state.v){ override val isSilent: Boolean get() = true })
            obj.addAction(object : Marker(state.tau) { override val name get() = "Appear" })
            changes.objects.add(obj to State(state.r.copy(t = now), V3_0, state.tau - (state.r.t - now)))
        } else
            changes.objects.add(obj to state)
    }

    override fun stateInFrame(obj: Obj, frame: Frame) = space.state(obj).transform(origin, frame)

    override fun deactivate(action: Action<*>) {
        base.deactivate(action)
    }

    override fun complete(action: Action<*>) {
        changes.completions.add(action)
    }
}

class Space(val states: MutableMap<Obj, State> = mutableMapOf()) {
    fun set(obj: Obj, state: State) {
        states[obj] = state
    }

    fun state(obj: Obj) =
        (states[obj] ?: throw IllegalArgumentException("unknown object $obj"))

    fun copy() = Space(states.toMutableMap())
}

class World(
    var now: Double = 0.0,
    private val theObjects: MutableSet<Obj> = mutableSetOf(),
    private val theCompleteActions: MutableSet<Action<*>> = mutableSetOf(),
    private val theActiveActions: MutableMap<Action<*>, Obj> = mutableMapOf(),

    private val actionStates: MutableMap<Action<*>, Any?> = mutableMapOf(),
    private val theEvents: MutableList<Event> = mutableListOf(),
    val space: Space = Space()
) : WorldView {
    override fun addActiveAction(action: Action<*>, obj: Obj) {
        theActiveActions[action] = obj
    }

    override val completeActions: Set<Action<*>> get() = theCompleteActions
    override val objects: Set<Obj> get() = theObjects
    override val activeActions: Map<Action<*>, Obj> get() = theActiveActions
    override val events: List<Event>
        get() = theEvents
    override var origin = Frame.ORIGIN
    override var logActions = true
    var logMotions = true

    override fun stateInFrame(obj: Obj, frame: Frame) =
        space.state(obj).transform(origin, frame)

    override fun actionState(action: Action<*>): Any? = actionStates[action] ?: action.init()

    override fun setAState(action: Action<*>, state: Any?) {
        actionStates[action] = state
    }

    override fun addEvent(e: Event) {
        theEvents.add(e)
    }

    override fun <T> addAction(obj: Obj, action: Action<T>) {
        obj.addAction(action as Action<*>)
    }

    override fun addMotion(obj: Obj, motion: Motion) {
        obj.addMotion(motion)
    }

    override fun complete(action: Action<*>) {
        theCompleteActions.add(action)
    }

    override fun deactivate(action: Action<*>) {
        theActiveActions.remove(action)
    }

    override fun addOrSetObject(obj: Obj, state: State) {
        if (state.r.t != now)
            throw IllegalArgumentException("World $this does't support adding objects at other times than now (=$now)")

        theObjects.add(obj)
        space.set(obj, state)
    }

    fun copyWith(newSpace: Space, newNow: Double) = World(
        newNow,
        objects.toMutableSet(),
        completeActions.toMutableSet(),
        activeActions.toMutableMap(),
        actionStates.toMutableMap(),
        events.toMutableList(),
        newSpace
    )

    override fun toString() = "time=$now ${objects.size} objects ${events.size} events"
}


data class Changes(
    val actions: MutableSet<Pair<Obj, Action<*>>> = mutableSetOf(),
    val actionStates: MutableMap<Action<*>, Any?> = mutableMapOf(),
    val completions: MutableSet<Action<*>> = mutableSetOf(),
    val motions: MutableList<Pair<Obj, Motion>> = mutableListOf(),
    val objects: MutableSet<Pair<Obj, State>> = mutableSetOf(),
    val events: MutableSet<Event> = mutableSetOf()
) {
    fun applyChanges(world: World) {
        for (entry in actions) {
            entry.first.addAction(entry.second)
        }
        for (entry in actionStates) {
            world.setAState(entry.key, entry.value)
        }

        for (entry in motions) {
            entry.first.addMotion(entry.second)
        }
        for (entry in objects) {
            assert(entry.second.r.t == world.now)
            world.addOrSetObject(entry.first, entry.second)
        }
        for (entry in completions) {
            world.complete(entry)
            world.deactivate(entry)
        }
        events.forEach { world.addEvent(it) }
    }
}

/**
 * Clone the given object, i.e. adding an object with the same state. Optionally modifying it.
 * @param obj to clone
 * @param name to use (defaulting to name plus "Clone"
 * @param vRelative relative velocity to given object (defaulting to zero, i.e. same velocity)
 * @param tau proper clock time of clone, null meaning same time as obj which is the default.
 * @return the clone object
 */
fun WorldView.cloneObj(
    obj: Obj,
    name: String = obj.name + "Clone",
    vRelative: Vector3 = V3_0,
    tau: Double? = null
): Obj {
    val clone = Obj(name)
    val state = stateInFrame(obj)
    val tauClone = tau ?: state.tau
    addOrSetObject(clone, State(state.r, state.v + vRelative, tauClone))
    addEvent(Event("Clone", obj, state.r, obj, state.tau, clone, tauClone))
    return clone
}

/**
 * @param obj to add at
 * @param r initial position (at world simulation coordinate time [now]) (defaulting to origin) with
 * @param v initial velocity relative to origin (defaulting zero) and
 * @param tau initial proper time of object clock (defaulting to 0)
 */
fun WorldView.addObj(obj: Obj, r: Vector4 = V4_0, v: Vector3 = V3_0, tau: Double = 0.0) {
    this.addOrSetObject(obj, State(r, v, tau))
}

/**
 * @return a frame coming with the given object at the current time
 */
fun WorldView.comovingFrame(obj: Obj) = this.stateInFrame(obj).let { Frame(it.r, it.v) }

