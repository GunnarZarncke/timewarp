package de.zarncke.timewarp

import de.zarncke.timewarp.math.*
import org.junit.Test
import kotlin.test.assertEquals

class TimeWarpTest {

    @Test
    fun testStateTransform() {
        val f0 = Frame.ORIGIN
        val fx = Frame(EX.to4(0.0), V3_0)
        val fxt = Frame(EX.to4(1.0), V3_0)
        val fdx = Frame(V4_0, EX * 0.5)
        val fxdx = Frame(EX.to4(0.0), EX * 0.5)
        val fxtdx = Frame(EX.to4(1.0), EX * 0.5)
        val fxtdy = Frame(EX.to4(1.0), EY * 0.5)
        val fytdx = Frame(EY.to4(1.0), EX * 0.5)

        val s0 = State(V4_0, V3_0, 0.0)

        assertEqualsS(s0, s0.transform(f0, f0), "identity")

        assertEqualsV((-EX).to4(0.0), s0.transform(f0, fx).r, "just displaced")

        assertEqualsV(lorentzTransform(EX * 0.5, V4_0), s0.transform(f0, fdx).r, "just moving")

        assertEqualsV(
            lorentzTransform(EX * 0.5, (-EX).to4(0.0)),
            s0.transform(f0, fxdx).r,
            "moving and displaced same dir"
        )

        assertEqualsV(lorentzTransform(EX * 0.5, (-EX).to4(-1.0)), s0.transform(f0, fxtdx).r, "general one-dim case")

        val s1 = State(EX.to4(0.0), V3_0, 0.0)
        val s2 = State(EY.to4(0.0), V3_0, 0.0)
        val s3 = State(
            (EX + EY).to4(0.0),
            V3_0, 0.0
        )

        // try lots of combinations
        for (state in setOf(s0, s1, s2, s3))
            for (frame in setOf(f0, fx, fxt, fxdx, fxtdx, fxtdy, fytdx)) {

                assertEqualsS(state, state.transform(frame, frame), "identity")

                val stateLoop = state.transform(fxtdy, frame).transform(frame, fytdx).transform(fytdx, fxtdy)
                assertEqualsS(state, stateLoop, "loop over $frame")
            }
    }

}

