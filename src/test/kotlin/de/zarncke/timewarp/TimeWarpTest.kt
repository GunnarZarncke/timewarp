package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class TimeWarpTest {

    @Test
    fun testStateTransform() {
        val f0 = Frame.ORIGIN
        val fx = Frame(EX.to4(0.0), V3_0)
        val fxt = Frame(EX.to4(1.0), V3_0)
        val fdx = Frame(V4_0, EX * 0.5)
        val fxdx = Frame(EX.to4(0.0), EX * 0.5)
        val fxtdx = Frame(EX.to4(1.0), EX * 0.5)
        val fxtdy = Frame(EX.to4(1.0), EY * 0.5)
        val fytdx = Frame(EY.to4(1.0), EX * 0.5)

        val s0 = State(V4_0, V3_0, 0.0)

        assertEqualsS(s0, s0.transform(f0, f0), "identity")

        assertEqualsV((-EX).to4(0.0), s0.transform(f0, fx).r, "just displaced")

        assertEqualsV(lorentzTransform(EX * 0.5, V4_0), s0.transform(f0, fdx).r, "just moving")

        assertEqualsV(
            lorentzTransform(EX * 0.5, (-EX).to4(0.0)),
            s0.transform(f0, fxdx).r,
            "moving and displaced same dir"
        )

        assertEqualsV(lorentzTransform(EX * 0.5, (-EX).to4(-1.0)), s0.transform(f0, fxtdx).r, "general one-dim case")

        val s1 = State(EX.to4(0.0), V3_0, 0.0)
        val s2 = State(EY.to4(0.0), V3_0, 0.0)
        val s3 = State(
            (EX + EY).to4(0.0),
            V3_0, 0.0
        )

        // try lots of combinations
        for (state in setOf(s0, s1, s2, s3))
            for (frame in setOf(f0, fx, fxt, fxdx, fxtdx, fxtdy, fytdx)) {

                assertEqualsS(state, state.transform(frame, frame), "identity")

                val stateLoop = state.transform(fxtdy, frame).transform(frame, fytdx).transform(fytdx, fxtdy)
                assertEqualsS(state, stateLoop, "loop over $frame")
            }
    }


    @Test
    fun testSimulateTrivial() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        o1.addMotion(Inertial(0.0, 1.0))

        assertEquals(State(V4_0, V3_0, 0.0), world.stateInFrame(o1, world.origin))
        tw.simulateTo(1.0)
        println(world.events)
        assertEquals(2, world.events.size)
        assertEquals("Motion:Inertial(0.0-1.0)", world.events[0].name)
        assertEquals("Motion-end:Inertial(0.0-1.0)", world.events[1].name)
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1, world.origin))
    }

    @Test
    fun testSimulateImpliedInertial() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)

        tw.simulateTo(1.0)
        assertEquals(0, world.events.size, "implied motions is not recorded")
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), world.stateInFrame(o1, world.origin))
    }

    @Test
    fun testSimulateSimpleMove() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = Obj("Test")
        val v = EX * 0.5
        tw.addObj(o1, V3_0, v)

        tw.simulateTo(1.0)
        println(world.events)
        assertEquals(State(v.to4(1.0), v, 1.0 / gamma(0.5)), world.stateInFrame(o1, world.origin))
    }

    @Test
    fun testSimulateSimpleAbruptMove() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val v = EX * 0.5
        o1.addMotion(AbruptVelocityChange(0.0, v))

        tw.simulateTo(1.0)
        assertEquals(1, world.events.size, "zero time moves record only one event")
        println(world.events)
        assertEquals(State(v.to4(1.0), v, 1.0 / gamma(0.5)), world.stateInFrame(o1, world.origin))
    }

    @Test
    fun testSimulateAcceleration() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val a = EX
        o1.addMotion(LongitudinalAcceleration(0.0, 1.0, a))

        tw.simulateTo(1.0)
        val s = relativisticCoordAcceleration(a, 1.0)
        assertEquals(s, world.stateInFrame(o1, world.origin))
        assertEquals(1, world.events.size, "we didn't reach end of acceleration")
        assertEquals("Motion:LongitudinalAcceleration(0.0-1.0) a=Vector3(x=1.0, y=0.0, z=0.0)", world.events[0].name)

        tw.simulateTo(2.0)
        val s2 = relativisticAcceleration(a, 1.0)
        println(world.events)
        assertEquals(2, world.events.size)
        assertStartsWith("Motion-end:LongitudinalAcceleration", world.events[1].name)
        assertEqualsV(s2.r, world.events[1].position)
        assertEquals(2.0, world.stateInFrame(o1, world.origin).r.t)
        // we don't check the inertial movement after the acceleration here
    }

}
