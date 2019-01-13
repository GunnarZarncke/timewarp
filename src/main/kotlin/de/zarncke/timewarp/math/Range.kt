package de.zarncke.timewarp.math

data class Range(val from: Double, val to: Double) : Comparable<Range> {
    override fun compareTo(other: Range) = comparator.compare(this, other)

    companion object {
        val comparator = compareBy(Range::from, Range::to)
    }
}
