package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Assert
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
    fun testSimulateActionMotionMix() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        val o2 = Obj("Two")
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX)
        o1.addAction(DetectCollision(0.0, 3.0, o2))
        o1.addMotion(LongitudinalAcceleration(1.0, 2.0, EX))
        o2.addAction(Marker(0.5))

        tw.simulateTo(3.0)
        val world = tw.theWorld
    }

    @Test
    fun testSimulateMotionWithActions() {
        val tw = TimeWarp()
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val a = EX
        o1.addMotion(LongitudinalAcceleration(0.0, 1.0, a))
        o1.addAction(Marker(0.0))
        o1.addAction(Pulse("beep", 0.0))

        tw.simulateTo(1.0)
        val world = tw.theWorld
        tw.simulateTo(1.0)
        val s = relativisticCoordAcceleration(a, 1.0)
        assertEqualsS(s, world.stateInFrame(o1), "expect accelerated motion so far")
    }

    @Test
    fun testSimulateAbruptMoveWithAction() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val v = EX * 0.5
        o1.addMotion(AbruptVelocityChange(1.0, v))
        o1.addAction(Marker(1.0))  // this caused the motion to be executed twise

        tw.simulateTo(2.0)
        println(world.events)
        assertEqualsS(State(v.to4(2.0), v, 1.0 + 1.0 / gamma(v)), world.stateInFrame(o1))
    }

    @Test
    fun testSimulateAbruptMoveWithExplicitInertial() {
        val tw = TimeWarp()
        val world = tw.theWorld
        val o1 = Obj("Test")
        tw.addObj(o1, V3_0)
        val v = EX * 0.5
        o1.addMotion(AbruptVelocityChange(1.0, v))
        o1.addMotion(Inertial(1.0, 2.0))

        tw.simulateTo(2.0)
        println(world.events)
        assertEqualsS(State(v.to4(2.0), v, 1.0 + 1.0 / gamma(v)), world.stateInFrame(o1))
    }

    @Test
    fun testRelativisticAberationConsistency() {
        val tw = TimeWarp()
        val o1 = Obj("Emitter at Origin")
        tw.addObj(o1, V3_0)
        val o2 = Obj("Receiver displces in y moving in x direction")
        val v = EX * 0.5
        tw.addObj(o2, EY, v)
        o1.addAction(object : Pulse("signal", 0.0) {
            override fun strike(world: WorldView, source: Obj, sourcePos: State, receiver: Obj, receiverObjPos: State) {
                val diff = receiverObjPos.r - sourcePos.r
                val receptionAngleSender = angle(EX, diff.to3())
                val receiveDirectionInReceiverFrame = diff.transform(world.origin, world.comovingFrame(receiver))
                val receptionAngleReceiver = angle(EX, receiveDirectionInReceiverFrame.to3())

                Assert.assertEquals("aberation agrees",receptionAngleReceiver,aberation(receptionAngleSender, v), eps)
                super.strike(world, source, sourcePos, receiver, receiverObjPos)
            }
        })

        tw.simulateTo(10.0)
        val world = tw.theWorld
        assertEquals("", world.events[0].name)
    }

}

