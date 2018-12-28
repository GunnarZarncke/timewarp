package de.zarncke.timewarp

import de.zarncke.timewarp.TimeWarp.Companion.V3_0
import de.zarncke.timewarp.TimeWarp.Companion.V4_0
import koma.cumsum
import koma.randn
import org.junit.Assert
import org.junit.Test
import kotlin.test.assertEquals
import java.lang.IllegalArgumentException
import kotlin.math.cosh
import kotlin.math.sinh
import kotlin.math.sqrt
import kotlin.math.tanh
import kotlin.test.fail

class TimeWarpTest {

    companion object {
        val eps = 0.001
        val EX = Vector3(1.0, 0.0, 0.0)
        val EY = Vector3(0.0, 1.0, 0.0)
        val EZ = Vector3(0.0, 0.0, 1.0)
    }

    @Test
    fun testSimulateT1() {
        val tw = TimeWarp()
        val world = tw.world
        val o1 = TimeWarp.Obj("Test")
        tw.addObj(o1, V4_0)
        o1.addAction(TimeWarp.Motion(0.0, EX.to4(1.0)))

        assertEquals(V4_0, o1.positionInFrame(world.origin))
        tw.simulateTo(1.0)
        assertEquals(EX.to4(1.0), o1.positionInFrame(world.origin))
    }

    @Test
    fun testSimulateRocketClocks() {
        val tw = TimeWarp()
        val o1 = TimeWarp.Obj("RocketBottom")
        o1.addAction(TimeWarp.Accelerate(0.0, EX, Double.POSITIVE_INFINITY))
        o1.addAction(TimeWarp.Pulse("A", 0.0, 1.0))
        tw.addObj(o1, V4_0)

        val o2 = TimeWarp.Obj("RocketTop")
        o2.addAction(TimeWarp.Accelerate(0.0, EX, Double.POSITIVE_INFINITY))
        tw.addObj(o2, EX.to4(0.0))

        val world = tw.simulateTo(10.0)
        val event = world.events[0]
        assertEquals("A0", event.name)
        assertEqualsV(EX.to4(0.0), event.position)
    }

    @Test
    fun testAddVelocity() {
        // stationary frame
        assertEqualsV(V3_0, observedAddedVelocity(V3_0, V3_0))
        assertEqualsV(EX * 0.5, observedAddedVelocity(V3_0, EX * 0.5))
        assertEqualsV(EX, observedAddedVelocity(V3_0, EX))

        // orthogonal
        assertEqualsV(Vector3(0.5, 0.433, 0.0), observedAddedVelocity(EX * 0.5, EY * 0.5))
        // same dir
        assertEqualsV(Vector3(0.8, 0.0, 0.0), observedAddedVelocity(EX * 0.5, EX * 0.5))

        // light speed
        // frame difference
        assertThrows<IllegalArgumentException> { observedAddedVelocity(EX, V3_0) }
        assertThrows<IllegalArgumentException> { observedAddedVelocity(EX, EX * 0.5) }
        // orthogonal
        assertEqualsV(Vector3(0.5, sqrt(3.0 / 4.0), 0.0), observedAddedVelocity(EX * 0.5, EY))
        // same dir
        assertEqualsV(EX, observedAddedVelocity(V3_0, EX))
    }


    @Test
    fun testLorentzTransform() {
        // stationary frame
        assertEqualsV(V4_0, lorentzTransform(V3_0, V4_0))
        assertEqualsV(V3_0.to4(1.0), lorentzTransform(V3_0, V3_0.to4(1.0)))
        assertEqualsV(EX.to4(0.0), lorentzTransform(V3_0, EX.to4(0.0)))
        assertEqualsV(EY.to4(0.0), lorentzTransform(V3_0, EY.to4(0.0)))
        assertEqualsV(EZ.to4(0.0), lorentzTransform(V3_0, EZ.to4(0.0)))
        assertEqualsV((EX + EY + EZ).to4(1.0), lorentzTransform(V3_0, (EX + EY + EZ).to4(1.0)))


        // same dir
        assertEqualsV(Vector4(-1 / sqrt(3.0), 2 / sqrt(3.0), 0.0, 0.0), lorentzTransform(EX * 0.5, EX.to4(0.0)))
        assertEqualsV(Vector4(1 / sqrt(3.0), 1 / sqrt(3.0), 0.0, 0.0), lorentzTransform(EX * 0.5, EX.to4(1.0)))

        // orthogonal
        assertEqualsV(EY.to4(0.0), lorentzTransform(EX * 0.5, EY.to4(0.0)))
        assertEqualsV(Vector4(1 / sqrt(3.0), -0.5 / sqrt(3.0), 1.0, 0.0), lorentzTransform(EX * 0.5, EY.to4(0.5)))
        assertEqualsV(Vector4(2 / sqrt(3.0), -1.0 / sqrt(3.0), 1.0, 0.0), lorentzTransform(EX * 0.5, EY.to4(1.0)))
    }

    @Test
    fun testAcceleration() {
        // stationary frame
        assertEqualsV(V4_0, relativisticAcceleration(V3_0, 0.0).first)
        assertEqualsV(V3_0, relativisticAcceleration(V3_0, 0.0).second)
        assertEqualsV(V3_0.to4(1.0), relativisticAcceleration(V3_0, 1.0).first)
        assertEqualsV(V3_0, relativisticAcceleration(V3_0, 1.0).second)

        // zero time acceleration
        assertEqualsV(V4_0, relativisticAcceleration(EX, 0.0).first)
        assertEqualsV(V3_0, relativisticAcceleration(EX, 0.0).second)

        assertEqualsV((EX * (cosh(1.0) - 1)).to4(sinh(1.0)), relativisticAcceleration(EX, 1.0).first)
        assertEqualsV(EX * tanh(1.0), relativisticAcceleration(EX, 1.0).second)
    }


    fun assertEqualsV(v1: Vector3, v2: Vector3) {
        Assert.assertEquals("x different in $v1!=$v2", v1.x, v2.x, eps)
        Assert.assertEquals("y different in $v1!=$v2", v1.y, v2.y, eps)
        Assert.assertEquals("z different in $v1!=$v2", v1.z, v2.z, eps)
    }

    fun assertEqualsV(v1: Vector4, v2: Vector4) {
        Assert.assertEquals("t different in $v1!=$v2", v1.t, v2.t, eps)
        assertEqualsV(v1.to3(), v2.to3())
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
