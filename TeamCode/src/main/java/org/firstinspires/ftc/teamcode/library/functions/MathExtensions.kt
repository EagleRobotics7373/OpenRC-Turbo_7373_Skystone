package org.firstinspires.ftc.teamcode.library.functions

import android.graphics.Color
import com.qualcomm.robotcore.hardware.ColorSensor


fun Double.toDegrees() = this * 180 / Math.PI

fun Double.toRadians() = this * Math.PI / 180

fun Double.truncate(decimals: Byte) = String.format("%.${decimals}f", this).toDouble()

fun Double.rangeClip(lowerBound: Double, upperBound: Double):Double {
    return when {
        this >= upperBound -> upperBound
        this <= lowerBound -> lowerBound
        else -> this
    }
}

fun Double.reverseIf(conditional: Boolean) = if (conditional) -this else this

fun Double.rangeBuffer(lowerBound: Double, upperBound: Double, defaultValue: Double):Double {
    return if (this in lowerBound..upperBound) defaultValue else this
}

fun Double.upperLimit(limitAt: Double) = if (this > limitAt) limitAt else this

fun Double.withinRange(target: Double, range: Double) = this in target-range..target+range

val ColorSensor.rhue : Double
    get() {
        return this.hsv[0].toDouble()
    }

val ColorSensor.rsaturation : Double
    get() {
        return this.hsv[1].toDouble()
    }

val ColorSensor.rvalue : Double
    get() {
        return this.hsv[2].toDouble()
    }

val ColorSensor.hsv : FloatArray
    get() {
        val SCALE_FACTOR = 255
        var hsvValues = FloatArray(3)
        Color.RGBToHSV(this.red()*SCALE_FACTOR, this.green()*SCALE_FACTOR, this.blue()*SCALE_FACTOR, hsvValues)
        return hsvValues
    }

