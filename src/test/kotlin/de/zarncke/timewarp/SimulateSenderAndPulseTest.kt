package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class SimulateSenderAndPulseTest {


    @Test
    fun testSimulatePulse() {
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
        assertEquals(EX.to4(1.0), world.events[1].position)
        assertEquals("Action:Pulsex:", world.events[1].name)
    }

}

