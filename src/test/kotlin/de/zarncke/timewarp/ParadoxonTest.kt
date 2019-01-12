package de.zarncke.timewarp

import org.junit.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ParadoxonTest {

    @Test
    fun testSimulateRocketClocks() {
        val tw = TimeWarp()
        val o1 = TimeWarp.Obj("RocketBottom")
        o1.addMotion(TimeWarp.LongitudinalAcceleration(0.0, Double.POSITIVE_INFINITY, EX))
        o1.addAction(TimeWarp.Sender("A", 0.0, 1.0))
        tw.addObj(o1, V4_0)

        val o2 = TimeWarp.Obj("RocketTop")
        o2.addMotion(TimeWarp.LongitudinalAcceleration(0.0, Double.POSITIVE_INFINITY, EX))
        tw.addObj(o2, EX.to4(0.0))

        val world = tw.simulateTo(10.0)
        val event = world.events[0]
        assertEquals("A0", event.name)
        assertEqualsV(EX.to4(0.0), event.position)
        println( tw.world.stateInFrame(o1, world.origin))
    }

    /**
     * Rocket R has length 2 and moves with 0.9c relative to two "doors" A and B. A and B are length 1 apart.
     * R moves first thru A and then thru B. Door B opens at the same time as A closes
     * (from the perspective of the inertial frame or the doors).
     */
    @Test
    fun testRocketFitsThruSmallGap() {
        val tw = TimeWarp()
        val rL = TimeWarp.Obj("RocketLeft")
        rL.addMotion(TimeWarp.AbruptVelocityChange(0.0,EX * 0.9))
        tw.addObj(rL, V4_0)
        val rR = TimeWarp.Obj("RocketRight")
        rR.addMotion(TimeWarp.AbruptVelocityChange(0.0,EX * 0.9))
        tw.addObj(rR, (EX * 2.0).to4(0.0))

        val dA = TimeWarp.Obj("DoorA")
        tw.addObj(dA, (EX * 4.0).to4(0.0))
        val dB = TimeWarp.Obj("DoorB")
        tw.addObj(dB, (EX * 5.0).to4(0.0))

        val world = tw.simulateTo(10.0)
        val event = world.events[0]
        assertEquals("A0", event.name)
        assertEqualsV(EX.to4(0.0), event.position)
    }

    @Test
    fun testTwinparadox() {
        val tw = TimeWarp()
        val twinOld = TimeWarp.Obj("TwinOld")
        tw.addObj(twinOld, V4_0)

        val twinYoung = TimeWarp.Obj("TwinYoung")
        twinYoung.addMotion(TimeWarp.LongitudinalAcceleration(0.0, 10.0, EX))
        twinYoung.addAction(TimeWarp.DetectCollision(10.0, 20.0, setOf(twinOld)))
        twinYoung.addMotion(TimeWarp.LongitudinalAcceleration(10.0, 20.0, EX * -1.0))
        tw.addObj(twinYoung, V4_0)

        val world = tw.simulateTo(10.0)
        val event = world.events[0]
        assertEquals("collision", event.name)
        val ageYoung = world.stateInFrame(twinYoung, world.origin).tau
        val ageOld = world.stateInFrame(twinOld, world.origin).tau
        assertTrue(ageOld > ageYoung)
    }


}
