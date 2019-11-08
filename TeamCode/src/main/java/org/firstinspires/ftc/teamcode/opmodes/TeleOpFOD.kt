package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.library.functions.rangeBuffer
import org.firstinspires.ftc.teamcode.library.functions.reverseIf
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController

@TeleOp(name="TeleOp FOD", group="basic")
class TeleOpFOD : TeleOpRD() {
    var fod = false
    var zeroAngle = 0.0
    lateinit var imuController:IMUController

    override fun init() {
        super.init()
        imuController = IMUController(hardwareMap)



    }

    override fun controlDrivetrain() {
        if (gamepad1.x) {
            fod = true
            zeroAngle = imuController.getHeading()
        }
        else if (gamepad1.a or gamepad1.b) fod = false


        if (fod)
        {
            val x = gamepad1.left_stick_x.toDouble().rangeBuffer(-0.15, 0.15, 0.0)
            val y = -gamepad1.left_stick_y.toDouble().rangeBuffer(-0.15, 0.15, 0.0)
            val z = gamepad1.right_stick_x.toDouble().rangeBuffer(-0.15, 0.15, 0.0)
            robot.holonomic.runWithoutEncoderVectored(x, y, z, zeroAngle - imuController.getHeading())
        } else super.controlDrivetrain()

        telemetry.addData("fod", fod)
        telemetry.addData("zero angle", zeroAngle)
    }

    private fun controlMusic() {
    }

}



