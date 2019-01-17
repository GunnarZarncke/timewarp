package de.zarncke.timewarp.math

import de.zarncke.timewarp.*
import org.junit.Assert
import org.junit.Test
import java.lang.IllegalArgumentException
import kotlin.math.*
import kotlin.test.assertEquals

class RelativisticMathTest {

    @Test
    fun testSeparation() {
        assertEquals(Separation.SPACELIKE, separation(V4_0, V3_0.to4(1.0)))
        assertEquals(Separation.LIGHTLIKE, separation(V4_0, EX.to4(1.0)))
        assertEquals(Separation.SPACELIKE, separation(V4_0, EX.to4(0.0)))

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
        assertEqualsV(
            Vector4(-1 / sqrt(3.0), 2 / sqrt(3.0), 0.0, 0.0),
            lorentzTransform(EX * 0.5, EX.to4(0.0))
        )
        assertEqualsV(
            Vector4(1 / sqrt(3.0), 1 / sqrt(3.0), 0.0, 0.0),
            lorentzTransform(EX * 0.5, EX.to4(1.0))
        )

        // orthogonal
        assertEqualsV(EY.to4(0.0), lorentzTransform(EX * 0.5, EY.to4(0.0)))
        assertEqualsV(
            Vector4(1 / sqrt(3.0), -0.5 / sqrt(3.0), 1.0, 0.0),
            lorentzTransform(EX * 0.5, EY.to4(0.5))
        )
        assertEqualsV(
            Vector4(2 / sqrt(3.0), -1.0 / sqrt(3.0), 1.0, 0.0),
            lorentzTransform(EX * 0.5, EY.to4(1.0))
        )

        // lorentz identities
        for (v4 in listOf(V4_0, V3_0.to4(1.0), EX.to4(0.0), EX.to4(1.0), EY.to4(1.0), EY.to4(1.0))) {
            val v = EX * 0.5
            assertEqualsV(v4, lorentzTransformInv(v, lorentzTransform(v, v4)))
        }

    }

    @Test
    fun testAcceleration() {
        // stationary frame
        assertEqualsV(V4_0, relativisticAcceleration(V3_0, 0.0).r)
        assertEqualsV(V3_0, relativisticAcceleration(V3_0, 0.0).v)
        assertEqualsV(V3_0.to4(1.0), relativisticAcceleration(V3_0, 1.0).r)
        assertEqualsV(V3_0, relativisticAcceleration(V3_0, 1.0).v)

        // zero time acceleration
        assertEqualsV(V4_0, relativisticAcceleration(EX, 0.0).r)
        assertEqualsV(V3_0, relativisticAcceleration(EX, 0.0).v)

        assertEqualsV((EX * (cosh(1.0) - 1)).to4(sinh(1.0)), relativisticAcceleration(EX, 1.0).r)
        assertEqualsV(EX * tanh(1.0), relativisticAcceleration(EX, 1.0).v)
    }

    @Test
    fun testAccelerationProperAndCoordinate() {
        val s1 = relativisticAcceleration(EX, 1.0)
        val s2 = relativisticCoordAcceleration(EX, s1.r.t)
        assertEqualsS(s1, s2, "relativistic acceleration should agree between proper and coordinate time")

        val s3 = relativisticCoordAcceleration(EX, 1.0)
        val s4 = relativisticAcceleration(EX, s3.tau)
        assertEqualsS(s3, s4, "relativistic acceleration should agree between coordinate and proper time")
    }

    @Test
    fun testNoAccelerationProperAndCoordinate() {
        val s1 = relativisticAcceleration(V3_0, 1.0)
        val s2 = relativisticCoordAcceleration(V3_0, s1.r.t)
        assertEqualsS(s1, s2, "relativistic acceleration should agree between proper and coordinate time")

        val s3 = relativisticCoordAcceleration(V3_0, 1.0)
        val s4 = relativisticAcceleration(V3_0, s3.tau)
        assertEqualsS(s3, s4, "relativistic acceleration should agree between coordinate and proper time")
    }

    @Test
    fun testAccelerationProperAndCoordinateBase() {
        val veps = Vector3(Double.MIN_VALUE, 0.0, 0.0)
        assertEqualsS(
            State(V4_0, V3_0, 0.0),
            relativisticCoordAcceleration(V3_0, 0.0, Frame(V4_0, V3_0))
        )
        assertEqualsS(
            State(V4_0, V3_0, 0.0),
            relativisticCoordAcceleration(veps, 0.0, Frame(V4_0, V3_0))
        )
        assertEqualsS(
            State(V4_0, V3_0, 0.0),
            relativisticCoordAcceleration(V3_0, 0.0, Frame(V4_0, veps))
        )
        assertEqualsS(
            relativisticCoordAcceleration(V3_0, 1.0),
            relativisticCoordAcceleration(V3_0, 1.0, Frame(V4_0, V3_0))
        )
        assertEqualsS(
            relativisticCoordAcceleration(veps, 1.0),
            relativisticCoordAcceleration(V3_0, 1.0, Frame(V4_0, V3_0))
        )
        assertEqualsS(
            relativisticCoordAcceleration(EX, 1.0),
            relativisticCoordAcceleration(EX, 1.0, Frame(V4_0, V3_0))
        )
        assertEqualsS(
            relativisticCoordAcceleration(EX, 1.0),
            relativisticCoordAcceleration(EX, 1.0, Frame(V4_0, veps))
        )
    }

    @Test
    fun testAccelerationProperAndCoordinateOrtho() {
        val a = 0.5
        val t = 1.0
        val v = 0.5
        val s = relativisticCoordAcceleration(EX * a, t, Frame(V4_0, EY * v)) // note EX and EY orthogonal directions
        assertEquals(1 / a * asinh(a * t / gamma(v)), s.tau)
        Assert.assertEquals("tau should transform back to t", 1.0, s.r.t, eps)
    }

    @Test
    fun testAccelerationProperAndCoordinateCollinear() {
        val a = 0.5
        val t = 1.0
        val v = 0.5
        val atg = a * t / gamma(v)
        val s = relativisticCoordAcceleration(EX * a, t, Frame(V4_0, EX * v))  // note EX in both cases
        assertEquals(1 / a * asinh((v * (-sqrt(atg * atg + 2 * v * atg + 1) + 1) + atg) / (1 - v * v)), s.tau)
        Assert.assertEquals("tau should transform back to t", 1.0, s.r.t, eps)
    }

    @Test
    fun testAccelerationProperAndCoordinateFull() {
        val a = 0.5
        val t = 1.0
        val v = 0.5
        val vv = (EX + EY).unit() * v   // v and a into different directions
        val atg = a * t / gamma(v)
        val w = EX.dot(vv)
        val s = relativisticCoordAcceleration(EX * a, t, Frame(V4_0, vv))
        assertEquals(1 / a * asinh((w * (-sqrt(atg * atg + 2 * w * atg + 1) + 1) + atg) / (1 - w * w)), s.tau)
        Assert.assertEquals("tau should transform back to t", 1.0, s.r.t, eps)
    }


    @Test
    fun testAccelerationProperAndCoordinateRelative() {
        val dv = EX * 0.5
        val f = Frame(V4_0, dv)
        val s1 = relativisticAcceleration(EX, 1.0).transform(f, Frame.ORIGIN)
        val s2 = relativisticCoordAcceleration(EX, s1.r.t, f)
        assertEqualsS(
            s1,
            s2,
            "relativistic acceleration should agree between proper and coordinate time"
        )

        val s3 = relativisticCoordAcceleration(EX, 1.0, f)
        val s4 = relativisticAcceleration(EX, s3.tau).transform(f, Frame.ORIGIN)
        assertEqualsS(
            s3,
            s4,
            "relativistic acceleration should agree between coordinate and proper time"
        )
    }

}