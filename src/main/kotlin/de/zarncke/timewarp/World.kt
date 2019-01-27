package de.zarncke.timewarp

import de.zarncke.timewarp.math.V3_0
import de.zarncke.timewarp.math.Vector3
import java.lang.IllegalArgumentException

interface WorldView {
    /**
     * @param obj to get position and velocity for (at current time t in world frame)
     * @param frame frame to get state in
     * @return state (position and velocity) in the given frame
     */
    fun stateInFrame(obj: Obj, frame: Frame = origin): State
    val origin: Frame
    val objects: List<Obj>
    val logActions: Boolean
}

class World(
    var now: Double = 0.0,
    override val objects: MutableList<Obj> = mutableListOf(),
    val completeActions: MutableSet<Action> = mutableSetOf(),
    val activeActions: MutableMap<Action, Obj> = mutableMapOf(),
    val events: MutableList<Event> = mutableListOf(),
    val states: MutableMap<Obj, State> = mutableMapOf()
) : WorldView {
    override var origin = Frame.ORIGIN
    override var logActions = true
    var logMotions = true

    /**
     * @param obj to add at
     * @param r initial position (at world simulation coordinate time [now]) (defaulting to origin) with
     * @param v initial velocity relative to origin (defaulting zero) and
     * @param tau initial proper time of object clock (defaulting to 0)
     */
    fun addObj(obj: Obj, r: Vector3 = V3_0, v: Vector3 = V3_0, tau: Double = 0.0) {
        objects.add(obj)
        set(obj, State(r.to4(now), v, tau))
    }

    fun comovingFrame(obj: Obj) =
        (states[obj] ?: throw IllegalArgumentException("unknown object $obj"))
            .let { Frame(it.r, it.v) }

    override fun stateInFrame(obj: Obj, frame: Frame) =
        (states[obj] ?: throw IllegalArgumentException("unknown object $obj")).transform(origin, frame)

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

    override fun toString() = "time=$now ${objects.size} objects ${events.size} events"
}



data class Changes(
    val actions: MutableSet<Pair<Obj, Action>> = mutableSetOf(),
    val completions: MutableSet<Action> = mutableSetOf(),
    val motions: MutableList<Pair<Obj, Motion>> = mutableListOf(),
    val objects: MutableSet<Pair<Obj, State>> = mutableSetOf(),
    val events: MutableSet<Event> = mutableSetOf()
) {
    /**
     * Clone the given object, i.e. adding an object with the same state. Optionally modifying it.
     * @param obj to clone
     * @param name to use (defaulting to name plus "Clone"
     * @param vRelative relative velocity to given object (defaulting to zero, i.e. same velocity)
     * @param tau proper clock time of clone, null meaning same time as obj which is the default.
     * @return the clone object
     */
    fun cloneObj(
        world: WorldView,
        obj: Obj,
        name: String = obj.name + "Clone",
        vRelative: Vector3 = V3_0,
        tau: Double? = null
    ): Obj {
        val clone = Obj(name)
        val state = world.stateInFrame(obj)
        val tauClone = tau ?: state.tau
        objects.add(clone to State(state.r, state.v + vRelative, tauClone))
        events.add(Event("Clone", state.r, obj, state.tau, clone, tauClone))
        return clone
    }

    fun applyChanges(world: World) {
        for (entry in actions) {
            entry.first.addAction(entry.second)
        }
        for (entry in motions) {
            entry.first.addMotion(entry.second)
        }
        for (entry in objects) {
            world.addObj(entry.first)
            assert(entry.second.r.t == world.now)
            world.set(entry.first, entry.second)
        }
        for (entry in completions) {
            world.completeActions.add(entry)
            world.activeActions.remove(entry)
        }
        world.events.addAll(events)
    }
}
