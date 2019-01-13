package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Assert
import org.junit.Test
import kotlin.test.assertEquals

class MotionTest {
    @Test
    fun testMotionInertialProper() {
        val move = Inertial(0.0, 2.0)

        // starting at the origin
        assertEquals(State(V4_0, V3_0, 0.0), move.moveUntilProperTime(Frame.ORIGIN, 0.0, 0.0))
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), move.moveUntilProperTime(Frame.ORIGIN, 0.0, 1.0))
        assertEquals(State(V3_0.to4(2.0), V3_0, 2.0), move.moveUntilProperTime(Frame.ORIGIN, 0.0, 2.0))

        // starting somewhere/time else
        val coMovingFrame1 = Frame(EX.to4(1.0), V3_0)
        assertEquals(State(V3_0.to4(-1.0), V3_0, 0.0), move.moveUntilProperTime(coMovingFrame1, 1.0, 0.0))
        assertEquals(State(V4_0, V3_0, 1.0), move.moveUntilProperTime(coMovingFrame1, 1.0, 1.0))
        assertEquals(State(V3_0.to4(1.0), V3_0, 2.0), move.moveUntilProperTime(coMovingFrame1, 1.0, 2.0))

        // starting at the origin, but already in motion
        val coMovingFrameEX = Frame(V4_0, EX * 0.5)
        assertEquals(State(V4_0, V3_0, 0.0), move.moveUntilProperTime(coMovingFrameEX, 0.0, 0.0))
        assertEquals(State(V3_0.to4(1.0), V3_0, 1.0), move.moveUntilProperTime(coMovingFrameEX, 0.0, 1.0))
        assertEquals(State(V3_0.to4(2.0), V3_0, 2.0), move.moveUntilProperTime(coMovingFrameEX, 0.0, 2.0))
    }

    @Test
    fun testMotionInertialCoord() {
        val move = Inertial(0.0, 2.0)

        assertEqualsS(State(V4_0, V3_0, 0.0), move.moveUntilCoordinateTime(Frame.ORIGIN, 0.0))
        assertEqualsS(State(V3_0.to4(1.0), V3_0, 1.0), move.moveUntilCoordinateTime(Frame.ORIGIN, 1.0))
        assertEqualsS(State(V3_0.to4(2.0), V3_0, 2.0), move.moveUntilCoordinateTime(Frame.ORIGIN, 2.0))
        assertEqualsS(State(V3_0.to4(2.0), V3_0, 2.0), move.moveUntilCoordinateTime(Frame.ORIGIN, 3.0))

        // starting somewhere/time else
        val coMovingFrame1 = Frame(EX.to4(1.0), V3_0)
        assertEqualsS(State(EX.to4(1.0), V3_0, 0.0), move.moveUntilCoordinateTime(coMovingFrame1, 1.0))
        assertEqualsS(State(EX.to4(2.0), V3_0, 1.0), move.moveUntilCoordinateTime(coMovingFrame1, 2.0))
        assertEqualsS(State(EX.to4(3.0), V3_0, 2.0), move.moveUntilCoordinateTime(coMovingFrame1, 3.0))
        assertEqualsS(State(EX.to4(3.0), V3_0, 2.0), move.moveUntilCoordinateTime(coMovingFrame1, 4.0))

        // starting in motion
        val v = EX * 0.5
        val gamma = gamma(0.5)
        val coMovingFrameEX = Frame(V4_0, v)

        assertEqualsS(State(V4_0, v, 0.0), move.moveUntilCoordinateTime(coMovingFrameEX, 0.0))
        assertEqualsS(State(v.to4(1.0), v, 1/gamma), move.moveUntilCoordinateTime(coMovingFrameEX, 1.0))
        assertEqualsS(State((v * 2.0 ).to4(2.0), v, 2/gamma), move.moveUntilCoordinateTime(coMovingFrameEX, 2.0))
        assertEqualsS(State((v * 2.0 * gamma).to4(2.0*gamma), v, 2.0), move.moveUntilCoordinateTime(coMovingFrameEX, 3.0))
    }

}
