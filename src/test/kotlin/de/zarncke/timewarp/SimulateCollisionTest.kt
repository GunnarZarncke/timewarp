package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class SimulateCollisionTest {


    @Test
    fun testTrivial() {
        val tw = TimeWarp()
        val v = EX * 0.5
        val o1 = Obj("A")
        val o2 = Obj("B")
        tw.addObj(o1, V3_0)
        tw.addObj(o2, EX, -v)
        o1.addAction(DetectCollision(0.0, Double.POSITIVE_INFINITY, o2))

        tw.simulateTo(2.0)

        val world = tw.theWorld
        println(world.events.joinToString("\n"))
        assertEquals(V4_0, world.events[0].receiverState.r)
        assertStartsWith("Action", world.events[0].name)
        assertEquals(V3_0.to4(2.0), world.events[1].receiverState.r)
        assertEquals("collide", world.events[1].name)
    }

}

