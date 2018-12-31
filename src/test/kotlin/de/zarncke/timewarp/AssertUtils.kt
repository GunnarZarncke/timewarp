package de.zarncke.timewarp

import org.junit.Assert
import kotlin.test.fail


fun assertEqualsV(v1: Vector3, v2: Vector3) {
    Assert.assertEquals("x different in $v1!=$v2", v1.x, v2.x, TimeWarpTest.eps)
    Assert.assertEquals("y different in $v1!=$v2", v1.y, v2.y, TimeWarpTest.eps)
    Assert.assertEquals("z different in $v1!=$v2", v1.z, v2.z, TimeWarpTest.eps)
}

fun assertEqualsV(v1: Vector4, v2: Vector4) {
    Assert.assertEquals("t different in $v1!=$v2", v1.t, v2.t, TimeWarpTest.eps)
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