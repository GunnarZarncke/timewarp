package de.zarncke.timewarp

@Deprecated("unused, was intended to capture all actions in formulas for review")
class Formula {

    val variables = mutableSetOf<String>()
    val equations = mutableListOf<String>()
    var v1 = 0
    var v2 = 0
    var t = 0

    fun varForV1(): String {
        val v = "v_$v1"; v1++; variables.add(v); return v
    }

    fun varForV2(): String {
        val v = "u_$v2"; v2++; variables.add(v); return v
    }

    fun varForTime(): String {
        val v = "t_$t"; t++; variables.add(v); return v
    }

    fun addVelocity(v: String, u: String): String {
        val w = varForV1()
        equations.add(
            "$w &= $v \\oplus  $u &= " +
                    "\\frac{1}{1 - $u \\cdot $v}\\left[$u - $v + \\frac{\\gamma_$v}{1+\\gamma_$v} $v \\times($v \\times $u)\\right]"
        )
        return w
    }

}