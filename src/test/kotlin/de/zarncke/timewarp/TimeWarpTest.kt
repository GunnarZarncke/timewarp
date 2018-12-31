package de.zarncke.timewarp

import org.junit.Test
import kotlin.test.assertEquals

class TimeWarpTest {

    @Test
    fun testStateTransform() {
        val f0 = Frame.ORIGIN
        val f1 = Frame(EX.to4(1.0), EY * 0.1)
        val f2 = Frame(EY.to4(1.0), EX * 0.1)

        val s1 = State((EX + EY).to4(0.0), V3_0, 0.0)
        val s2 = State((EX + EY).to4(0.0), V3_0, 0.0)
    }



    @Test
    fun testSimulateT1() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = TimeWarp.Obj("Test")
        tw.addObj(o1, V4_0)
        o1.addMotion(TimeWarp.Inertial(0.0, 1.0))

        assertEquals(State(V4_0, V3_0, 0.0), world.stateInFrame(o1, world.origin))
        tw.simulateTo(1.0)
        assertEquals(State(EX.to4(1.0), EX, 1.0), world.stateInFrame(o1, world.origin))
    }

}
