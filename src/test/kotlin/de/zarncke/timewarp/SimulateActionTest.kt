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
    fun testSimulateNeverAction() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        o1.addAction(Marker(2.0))

        tw.simulateTo(1.0)
        assertEquals(0, tw.theWorld.events.size)
        assertEquals(V3_0.to4(1.0), tw.theWorld.stateInFrame(o1).r)
    }

    @Test
    fun testSimulateSimpleAction() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        val v = EX * 0.5
        tw.addObj(o1, V3_0, v)
        val marker = Marker(0.5)
        o1.addAction(marker)

        tw.simulateTo(1.0)
        val world = tw.theWorld
        assertEquals((EX * 0.25 * gamma(v)).to4(0.5 * gamma(v)), world.events[0].receiverState.r)
        assertEquals(marker, world.events[0].cause)
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
        val marker = Marker(0.25)
        o1.addAction(object : Action<Unit>(0.5, 0.5) {
            override fun init() {}
            override fun act(world: WorldView, obj: Obj, tau: Double, state: Unit) {
                o2 = world.cloneObj(obj, "Spawned", v, 0.0)
                    .addAction(marker)
            }
        })

        tw.simulateTo(1.0)
        // events at t=0.5
        assertEquals(V3_0.to4(0.5), world.events[0].receiverState.r)
        assertEquals("Clone", world.events[1].name)
        assertEquals(o2, world.events[1].receiver)
        assertEquals(V3_0.to4(0.5), world.events[1].receiverState.r)

        assertEquals(marker, world.events[2].cause)
        assertEquals((v * 0.25 * gamma(v)).to4(0.5 + 0.25 * gamma(v)), world.events[2].receiverState.r)
        assertEquals(0.25, world.events[2].receiverState.tau)
        // state at t=1.0
        assertEqualsS(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1))
        assertEqualsS(State((v * 0.5).to4(1.0), v, 0.5 / gamma(v)), world.stateInFrame(o2!!))
    }

    @Test
    fun testAddDisplacedInertial() {
        val tw = TimeWarp()
        val left = Obj("LeftEnd")
        val right = Obj("RightEnd")
        tw.addObj(left, V3_0)
        left.addAction(AddDisplaced(0.0, right, EX))
        left.addAction(Marker(0.5))

        val world = tw.simulateTo(1.0)
        assertEqualsS(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(left))
        assertEqualsS(State(EX.to4(1.0), V3_0, 1.0), world.stateInFrame(right))
        assertEqualsV(V3_0.to4(0.5), tw.events(causeClass = Marker::class.java)[0].receiverState.r)
    }

    @Test
    fun testAddDisplacedMoving() {
        val tw = TimeWarp()
        val left = Obj("LeftEnd")
        val right = Obj("RightEnd")
        val v = EX * 0.5
        tw.addObj(left, V3_0, v)
        left.addAction(AddDisplaced(0.0, right, EX))
        left.addAction(Marker(0.0))
        right.addAction(Marker(0.0))

        val world = tw.simulateTo(2.0)
        println(tw.theWorld.events.joinToString("\n"))

        val r = lorentzTransformInv(v, EX.to4(0.0))

        val markers = tw.events(causeClass = Marker::class.java)
        assertEqualsV(V3_0.to4(0.0), markers[0].receiverState.r)
        assertEqualsV(r, markers[1].receiverState.r)
        assertEqualsS(State(EX.to4(2.0), v, 2.0 / gamma(v)), world.stateInFrame(left))
        assertEqualsS(
            State((r.to3() + v * (2 - r.t)).to4(2.0), v, (2.0 - r.t) / gamma(v)),
            world.stateInFrame(right)
        )
    }
}
