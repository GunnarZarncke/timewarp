package de.zarncke.timewarp

import org.junit.Test
import kotlin.test.assertEquals

class TimeWarpTest {

    @Test
    fun test(){
        val tw = TimeWarp()
        val o1 = Obj(tw.space)
        tw.addObj(o1)
        o1.addAction(Motion(0.0, Vector4(1.0, 1.0,0.0,0.0)))

        assertEquals(Vector4(0.0, 0.0,0.0,0.0), tw.position(o1))
        tw.simulateTo(1.0)
        assertEquals(Vector4(1.0, 1.0,0.0,0.0), tw.position(o1))
    }
}
