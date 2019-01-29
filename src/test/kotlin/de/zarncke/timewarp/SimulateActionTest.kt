package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class SimulateActionTest {


    @Test
    fun testSimulateLateAction() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        o1.addAction(Marker(2.0))

        tw.simulateTo(1.0)
        assertEquals(V3_0.to4(1.0), tw.theWorld.stateInFrame(o1).r)
    }

    @Test
    fun testSimulateSimpleAction() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        val v = EX * 0.5
        tw.addObj(o1, V3_0, v)
        o1.addAction(Marker(0.5))

        tw.simulateTo(1.0)
        val world = tw.theWorld
        println(world.events)
        assertEquals((EX * 0.25 * gamma(v)).to4(0.5 * gamma(v)), world.events[0].position)
        assertEquals("Action:Marker:Range(from=0.5, to=0.5)", world.events[0].name)
        assertEqualsS(State(v.to4(1.0), v, 1 / gamma(v)), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateActionCreateObject() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        var o2: Obj? = null
        tw.addObj(o1, V3_0)
        val v = EX * 0.5
        o1.addAction(object : Action<Unit>(0.5, 0.5) {
            override fun init() {}
            override fun act(world: WorldView, obj: Obj, tau: Double, t: Unit) {
                o2 = world.cloneObj(obj, "Spawned", v, 0.0)
                    .addAction(Marker(0.25))
            }
        })

        tw.simulateTo(1.0)
        println(world.events.joinToString("\n"))
        // events at t=0.5
        assertEquals(V3_0.to4(0.5), world.events[0].position)
        assertEquals("Clone", world.events[1].name)
        assertEquals(o2, world.events[1].receiver)
        assertEquals(V3_0.to4(0.5), world.events[1].position)

        assertStartsWith("Action:Marker:", world.events[2].name)
        assertEquals((v * 0.25 * gamma(v)).to4(0.5 + 0.25 * gamma(v)), world.events[2].position)
        assertEquals(0.25, world.events[2].tauReceiver)
        // state at t=1.0
        assertEqualsS(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1))
        assertEqualsS(State((v * 0.5).to4(1.0), v, 0.5 / gamma(v)), world.stateInFrame(o2!!))
    }
}

