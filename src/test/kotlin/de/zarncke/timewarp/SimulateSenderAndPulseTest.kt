package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.test.assertEquals

class SimulateSenderAndPulseTest {


    @Test
    fun testSimulatePulseTrivial() {
        val tw = TimeWarp()
        val o1 = Obj("Sender")
        val o2 = Obj("Receiver")
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX)
        val pulse = Pulse("beep", 0.0)
        o1.addAction(pulse)

        tw.simulateTo(2.0)

        val world = tw.theWorld
        println(world.events.joinToString("\n"))
        assertEquals(V4_0, world.events[0].receiverState.r)
        assertEquals(pulse, world.events[0].cause)
        assertEquals(V4_0, world.events[1].receiverState.r)
        assertEquals("beep", world.events[1].name)
        assertEquals(EX.to4(1.0), world.events[2].receiverState.r)
        assertEquals("beep", world.events[2].name)
    }

    @Test
    fun testSimulatePulseSimple() {
        val tw = TimeWarp()
        val o1 = Obj("Sender")
        val o2 = Obj("Receiver")
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX)
        o1.addAction(Pulse("beep", 0.0))

        tw.simulateTo(3.0)

        val world = tw.theWorld
        assertEqualsV(V4_0, world.events[1].receiverState.r)
        assertEquals("beep", world.events[1].name)
        assertEqualsV(EX.to4(1.0), world.events[2].receiverState.r)
        assertEquals("beep", world.events[2].name)
    }


    @Test
    fun testSimulatePulseMotion() {
        val tw = TimeWarp()
        val o1 = Obj("Sender")
        val o2 = Obj("Receiver")
        val v= EX*0.5
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX,-v)
        o1.addAction(Pulse("beep", 0.0))

        tw.simulateTo(2.0)

        val world = tw.theWorld
        println(world.events.joinToString("\n"))

        assertEqualsV(V4_0, world.events[1].receiverState.r)
        assertEquals("beep", world.events[1].name)
        assertEqualsV((EX*(2.0/3)).to4(2.0/3), world.events[2].receiverState.r)
        assertEquals("beep", world.events[2].name)
    }

    @Test
    fun testSimulateSender() {
        val tw = TimeWarp()
        val o1 = Obj("Sender")
        val o2 = Obj("Receiver")
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX)
        o1.addAction(Sender("beep", 0.0, 1.0))

        tw.simulateTo(3.0)
        val world = tw.theWorld
        println(world.events.joinToString("\n"))

        assertEquals("Action", world.events[0].name, "start sending")
        assertEqualsV(V4_0, world.events[0].receiverState.r )
        assertTrue(world.events[0].cause is Sender)


        val pulses = tw.events(nameRegex = "pulse:beep.*".toRegex(), receiver = o2)
        assertEquals("pulse:beep-0", pulses[0].name)
        assertEquals("pulse:beep-1", pulses[1].name)
        assertEquals("pulse:beep-2", pulses[2].name)
        assertEquals(3, pulses.size)
        assertEquals(EX.to4(1.0), pulses[0].receiverState.r)
        assertEquals(EX.to4(2.0), pulses[1].receiverState.r)
        assertEquals(EX.to4(3.0), pulses[2].receiverState.r)
    }


}

