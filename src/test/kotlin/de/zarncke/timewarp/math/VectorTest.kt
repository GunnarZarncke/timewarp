package de.zarncke.timewarp.math

import de.zarncke.timewarp.assertEqualsV
import de.zarncke.timewarp.math.*
import koma.mat
import koma.matrix.Matrix
import org.junit.Assert
import org.junit.Ignore
import org.junit.Test
import kotlin.test.assertEquals

class VectorTest {
    @Test
    fun testVectorRotation() {
        assertEqualsV(EX, EY.rotate(EZ, 90.deg2Rad()))
    }

    @Test
    fun testVectorRotationComplex() {
        val someVector = EX + EY * 2.0 + EZ * 3.0
        val someOtherVector = EX * 2.0 + EY * 3.0 + EZ * 4.0

        var fiveTimesRotated = someVector
        for (i in 1..5) fiveTimesRotated = fiveTimesRotated.rotate(someOtherVector, (360 / 5).deg2Rad())
        assertEqualsV(someVector, fiveTimesRotated)
    }

    /**
     * This just verifies that our matrix operations do the same thing as Koma's do.
     */
    @Ignore
    @Test
    fun testCompareKoma() {

        val tvec = EX + EY * 2.0 + EZ * 3.0
        val tmat = eulerAxisToRotMatrix(EZ, 90.deg2Rad())
        val tres = tvec * tmat


        val kvec = mat[tvec.x, tvec.y, tvec.z]
        val kmat = Matrix<Double>(3, 3) { r, c -> tmat.toArray()[r][c] }
        val kres = kvec * kmat

        assertEqualsV(tres, Vector3(kres.getDouble(0,0), kres.getDouble(0,1), kres.getDouble(0,2)))

        val ktres = mat[tres.x, tres.y, tres.z]
        assertEquals(kres, ktres)
    }

}
