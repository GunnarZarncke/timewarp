package de.zarncke.timewarp

import de.zarncke.timewarp.math.Vector4

data class Event(
    val name: String,
    val cause: Any?,
    val position: Vector4,
    val sender: Obj,
    val tauSender: Double,
    val receiver: Obj,
    val tauReceiver: Double
)