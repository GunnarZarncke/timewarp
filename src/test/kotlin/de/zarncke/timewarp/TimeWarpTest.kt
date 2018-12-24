package de.zarncke.timewarp

import koma.cumsum
import koma.randn
import org.junit.Assert
import org.junit.Test
import kotlin.test.assertEquals
import de.zarncke.timewarp.TimeWarp.Companion.observedAddedVelocity
import java.lang.IllegalArgumentException
import kotlin.test.fail

class TimeWarpTest {

    companion object {
        val eps = 0.001
    }

    @Test
    fun test() {
        val tw = TimeWarp()
        val o1 = TimeWarp.Obj(tw.space)
        tw.addObj(o1)
        o1.addAction(TimeWarp.Motion(0.0, Vector4(1.0, 1.0, 0.0, 0.0)))

        assertEquals(Vector4(0.0, 0.0, 0.0, 0.0), tw.position(o1))
        tw.simulateTo(1.0)
        assertEquals(Vector4(1.0, 1.0, 0.0, 0.0), tw.position(o1))
    }

    @Test
    fun testVelocity() {
        // stationary space
        assertEqualsV(Vector3(0.0, 0.0, 0.0), observedAddedVelocity(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)))
        assertEqualsV(Vector3(0.5, 0.0, 0.0), observedAddedVelocity(Vector3(0.0, 0.0, 0.0), Vector3(0.5, 0.0, 0.0)))
        assertEqualsV(Vector3(1.0, 0.0, 0.0), observedAddedVelocity(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0)))

        // orthogonal
        assertEqualsV(Vector3(0.5, 0.433, 0.0), observedAddedVelocity(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.5, 0.0)))
        // same dir
        assertEqualsV(Vector3(0.8, 0.0, 0.0), observedAddedVelocity(Vector3(0.5, 0.0, 0.0), Vector3(0.5, 0.0, 0.0)))

        // light speed
        // space difference
        assertThrows<IllegalArgumentException> { observedAddedVelocity(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)) }
        assertThrows<IllegalArgumentException> { observedAddedVelocity(Vector3(1.0, 0.0, 0.0), Vector3(0.5, 0.0, 0.0)) }
        // orthogonal
        assertEqualsV(Vector3(0.5, 0.866, 0.0), observedAddedVelocity(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 1.0, 0.0)))
        // same dir
        assertEqualsV(Vector3(1.0, 0.0, 0.0), observedAddedVelocity(Vector3(0.0, 0.0, 0.0), Vector3(1.0, 0.0, 0.0)))
    }

    fun assertEqualsV(v1: Vector3, v2: Vector3) {
        Assert.assertEquals("x different in $v1!=$v2", v1.x, v2.x, eps)
        Assert.assertEquals("y different in $v1!=$v2", v1.y, v2.y, eps)
        Assert.assertEquals("z different in $v1!=$v2", v1.z, v2.z, eps)
    }

    inline fun <reified T : Throwable> assertThrows(block: () -> Unit) {
        try {
            block()
            fail("expected block to throw exception ${T::class.simpleName}")
        } catch (t: Throwable) {
            if (t !is T)
                throw t
        }
    }
}
