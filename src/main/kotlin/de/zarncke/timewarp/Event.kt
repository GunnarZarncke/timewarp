package de.zarncke.timewarp

import de.zarncke.timewarp.math.Vector4

data class Event(
    val name: String,
    val cause: Cause<*>,
    val sender: Obj,
    val senderState: State,
    val receiver: Obj,
    val receiverState: State
){
    fun position() = senderState.r
}

interface Cause<T> : Comparable<T> {
    val name: String
    val isSilent: Boolean get() = false
}