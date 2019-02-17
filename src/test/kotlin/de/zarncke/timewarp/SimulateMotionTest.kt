package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class SimulateMotionTest {
    @Test
    fun testSimulateTrivial() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        o1.addMotion(Inertial(0.0, 1.0))

        assertEquals(State(V4_0, V3_0, 0.0), world.stateInFrame(o1))
        tw.simulateTo(1.0)
        println(world.events)
        assertEquals(2, world.events.size)
        assertEquals("Motion", world.events[0].name)
        assertEquals("Motion-end", world.events[1].name)
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateImpliedInertial() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)

        tw.simulateTo(1.0)
        assertEquals(0, world.events.size, "implied motions is not recorded")
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateSimpleMove() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        val v = EX * 0.5
        tw.addObj(o1, V3_0, v)

        tw.simulateTo(1.0)
        println(world.events)
        assertEquals(State(v.to4(1.0), v, 1.0 / gamma(0.5)), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateSimpleAbruptMove() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val v = EX * 0.5
        o1.addMotion(AbruptVelocityChange(0.0, v))

        tw.simulateTo(1.0)
        assertEquals(1, world.events.size, "zero time moves record only one event")
        println(world.events)
        assertEquals(State(v.to4(1.0), v, 1.0 / gamma(0.5)), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateMoveAndTurn() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        val v = EX * 0.5
        tw.addObj(o1, V3_0, v)
        o1.addMotion(AbruptVelocityChange(1.0, -v))  // means we should be inertial with origin again
        o1.addMotion(AbruptVelocityChange(2.0, -v))  // moving back
        o1.addMotion(AbruptVelocityChange(3.0, v))  // home

        tw.simulateTo(4.0)

        assertEquals(3, world.events.size, "expects events at velocity changes")
        println(world.events.joinToString(",\n"))

        val gamma = gamma(0.5)
        assertEqualsV((v * gamma).to4(gamma), world.events[0].receiverState.r)
        assertEqualsV((v * gamma).to4(gamma + 1), world.events[1].receiverState.r)
        assertEqualsV(V3_0.to4(2 * gamma + 1), world.events[2].receiverState.r)
        assertEqualsS(State(V3_0.to4(4.0), V3_0, 4.0 - (2 * gamma - 2)), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateAcceleration() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val a = EX
        val acceleration = LongitudinalAcceleration(0.0, 1.0, a)
        o1.addMotion(acceleration)

        tw.simulateTo(1.0)
        val s = relativisticCoordAcceleration(a, 1.0)
        assertEqualsS(s, world.stateInFrame(o1), "expect accelerated motion so far")
        assertEquals(1, world.events.size, "we didn't reach end of acceleration")
        assertEquals("Motion", world.events[0].name)
        assertEquals(acceleration, world.events[0].cause)

        tw.simulateTo(2.0)
        println(world.events)
        assertEquals(2, world.events.size)
        assertStartsWith("Motion-end", world.events[1].name)
        assertEquals(acceleration, world.events[1].cause)
        val s2 = relativisticAcceleration(a, 1.0)
        assertEqualsV(s2.r, world.events[1].receiverState.r, "expect an event at end of acceleration")
        assertEquals(2.0, world.stateInFrame(o1).r.t)
        // we don't check the inertial movement after the acceleration here
    }

    @Test
    fun testSimulateAccelerateAndTurn() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        val a = EX
        tw.addObj(o1, V3_0, V3_0)
        o1.addMotion(LongitudinalAcceleration(0.0, 1.0, a)) // accelerate
        o1.addMotion(LongitudinalAcceleration(1.0, 2.0, -a)) // decelerate until stop
        o1.addMotion(LongitudinalAcceleration(2.0, 3.0, -a)) // accelerate other direction
        o1.addMotion(LongitudinalAcceleration(3.0, 4.0, a)) // decelerate until back home

        tw.simulateTo(5.0)

        assertEquals(8, world.events.size, "expects 2 events per motion")
        println(world.events.joinToString(",\n"))

        val s = relativisticAcceleration(a, 1.0)
        assertEqualsV(V4_0, world.events[0].receiverState.r)
        assertEqualsV(s.r, world.events[1].receiverState.r)
        assertEquals(world.events[1].receiverState.r, world.events[2].receiverState.r)
        assertEqualsV((s.r.to3() * 2.0).to4(2 * s.r.t), world.events[3].receiverState.r)
        assertEquals(world.events[3].receiverState.r, world.events[4].receiverState.r)
        assertEquals(world.events[5].receiverState.r, world.events[6].receiverState.r)
        assertEqualsV(V3_0.to4(4 * s.r.t), world.events[7].receiverState.r)
        assertEqualsS(State(V3_0.to4(5.0), V3_0, 5.0 - 4 * (s.r.t - s.tau)), world.stateInFrame(o1))
    }

}

