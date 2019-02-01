package de.zarncke.timewarp

import de.zarncke.timewarp.math.EX
import de.zarncke.timewarp.math.V3_0
import org.junit.Test
import kotlin.test.assertEquals
import kotlin.test.assertTrue

class ParadoxonTest {

    @Test
    fun testSimulateRocketClocks() {
        val tw = TimeWarp()
        val o1 = Obj("RocketBottom")
        o1.addMotion(LongitudinalAcceleration(0.0, Double.POSITIVE_INFINITY, EX))
        o1.addAction(Sender("A", 0.0, 1.0))
        tw.addObj(o1, V3_0)

        val o2 = Obj("RocketTop")
        o2.addMotion(LongitudinalAcceleration(0.0, Double.POSITIVE_INFINITY, EX))
        tw.addObj(o2, EX)

        val world = tw.simulateTo(10.0)
        println(world.events.joinToString("\n"))

        val event = world.events[0]
        assertEquals("A0", event.name)
        assertEqualsV(EX.to4(0.0), event.position)
        println( world.stateInFrame(o1))
    }

    /**
     * Rocket R has length 2 and moves with 0.9c relative to two "doors" A and B. A and B are length 1 apart.
     * R moves first thru A and then thru B. Door B opens at the same time as A closes
     * (from the perspective of the inertial frame or the doors).
     */
    @Test
    fun testRocketFitsThruSmallGap() {
        val tw = TimeWarp()
        val rL = Obj("RocketLeft")
        tw.addObj(rL, V3_0, EX * 0.9)
        val rR = Obj("RocketRight")
        tw.addObj(rR, (EX * 2.0), EX * 0.9)

        val dA = Obj("DoorA")
        dA.addAction(DetectCollision(0.0,10.0, rL,rR))
        tw.addObj(dA, (EX * 4.0))
        val dB = Obj("DoorB")
        dB.addAction(DetectCollision(0.0,10.0, rL,rR))
        tw.addObj(dB, (EX * 5.0))

        val world = tw.simulateTo(10.0)
        println(world.events.joinToString("\n"))

        val event = world.events[0]
        assertEquals("DetectCollision", event.name)
        assertEqualsV(EX.to4(0.0), event.position)
    }

    @Test
    fun testTwinparadox() {
        val tw = TimeWarp()
        val twinOld = Obj("TwinOld")
        tw.addObj(twinOld, V3_0)

        val dt = 4.0
        val twinYoung = Obj("TwinYoung")
        twinYoung.addMotion(LongitudinalAcceleration(0.0, dt, EX))
        twinYoung.addAction(DetectCollision(dt, Double.POSITIVE_INFINITY, twinOld))
        twinYoung.addMotion(LongitudinalAcceleration(dt, 3*dt, -EX))
        twinYoung.addMotion(LongitudinalAcceleration(3*dt, 4*dt, EX))
        tw.addObj(twinYoung, V3_0)

        val world = tw.simulateTo(110.0)
        println(world.events.joinToString("\n"))

        val event = world.events[7]
        assertEquals("collide", event.name)
        val ageYoung = world.stateInFrame(twinYoung).tau
        val ageOld = world.stateInFrame(twinOld).tau
        assertTrue(ageOld > 6*ageYoung)
    }


}
