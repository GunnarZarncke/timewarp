package de.zarncke.timewarp

import org.junit.Assert
import kotlin.test.fail

const val EPS = 0.001


fun assertEqualsV(v1: Vector3, v2: Vector3, msg:String ="3-vector different") {
    Assert.assertEquals("$msg: x different in $v1!=$v2", v1.x, v2.x, EPS)
    Assert.assertEquals("$msg: y different in $v1!=$v2", v1.y, v2.y, EPS)
    Assert.assertEquals("$msg: z different in $v1!=$v2", v1.z, v2.z, EPS)
}

fun assertEqualsV(v1: Vector4, v2: Vector4, msg:String="4-vector different") {
    Assert.assertEquals("$msg: t different in $v1!=$v2", v1.t, v2.t, EPS)
    assertEqualsV(v1.to3(), v2.to3())
}

fun assertEqualsS(s1: State, s2: State, msg:String="states different") {
    Assert.assertEquals("$msg: tau different in $s1!=$s2", s1.tau, s2.tau, EPS)
    assertEqualsV(s1.r, s2.r, msg)
    assertEqualsV(s1.v, s2.v, msg)
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