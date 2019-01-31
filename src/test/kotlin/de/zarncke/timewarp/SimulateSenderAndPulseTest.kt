package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
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
        o1.addAction(Pulse("beep", 0.0))

        tw.simulateTo(2.0)

        val world = tw.theWorld
        println(world.events.joinToString("\n"))
        assertEquals(V4_0, world.events[0].position)
        assertStartsWith("Action:Pulse:", world.events[0].name)
        assertEquals(V4_0, world.events[1].position)
        assertEquals("beep", world.events[1].name)
        assertEquals(EX.to4(1.0), world.events[2].position)
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
        assertEqualsV(V4_0, world.events[1].position)
        assertEquals("beep", world.events[1].name)
        assertEqualsV(EX.to4(1.0), world.events[2].position)
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

        assertEqualsV(V4_0, world.events[1].position)
        assertEquals("beep", world.events[1].name)
        assertEqualsV((EX*(2.0/3)).to4(2.0/3), world.events[2].position)
        assertEquals("beep", world.events[2].name)
    }
}

