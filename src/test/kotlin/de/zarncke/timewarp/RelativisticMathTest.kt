package de.zarncke.timewarp

import org.junit.Test
import java.lang.IllegalArgumentException
import kotlin.math.cosh
import kotlin.math.sinh
import kotlin.math.sqrt
import kotlin.math.tanh
import kotlin.test.assertEquals

class RelativisticMathTest {

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

}