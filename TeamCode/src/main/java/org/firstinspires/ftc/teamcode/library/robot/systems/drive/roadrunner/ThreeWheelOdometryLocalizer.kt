package org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.OdometryConstants.*
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import org.firstinspires.ftc.teamcode.library.robot.robotcore.MisumiRobot


class ThreeWheelOdometryLocalizer(
        private val leftModule : ExpansionHubMotor,
        private val rightModule: ExpansionHubMotor,
        private val rearModule : ExpansionHubMotor,
        private val expansionHub: ExpansionHubEx,
        private val robot : MisumiRobot
)
    : ThreeTrackingWheelLocalizer(
        listOf(
//                Pose2d(INCH.fromCm(-2.2), INCH.fromCm(18.25), 0.0),
//                Pose2d(INCH.fromCm(-2.2), INCH.fromCm(-19.5), 0.0),
//                Pose2d(INCH.fromCm(-17.6), INCH.fromCm(0.0), Math.PI / 2)
                Pose2d(INCH.fromCm(leftXcm), INCH.fromCm(leftYcm), leftAngleDeg.toRadians()),
                Pose2d(INCH.fromCm(rightXcm), INCH.fromCm(rightYcm), rightAngleDeg.toRadians()),
                Pose2d(INCH.fromCm(rearXcm), INCH.fromCm(rearYcm), rearAngleDeg.toRadians())
        )
)
{
    val modulesExt = listOf(leftModule, rightModule, rearModule)

    val TICKS_PER_REVOLUTION = 8192
    val WHEEL_DIAMETER_mm = DistanceUnit.INCH.fromMm(38.0)

    init {
        modulesExt.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    override fun getWheelPositions(): List<Double> {
        val bulkData = expansionHub.bulkInputData

        val wheelPositions = emptyList<Double>().toMutableList()

        modulesExt.forEach {
            wheelPositions.add((if (reverseOutput) -1.0 else 1.0) * (bulkData.getMotorCurrentPosition(it).toDouble() / TICKS_PER_REVOLUTION) * WHEEL_DIAMETER_mm * Math.PI)
        }
        print("%% @OdometryLocalizer_REVExt   LEFT=${wheelPositions[0]}    RIGHT=${wheelPositions[1]}    REAR=${wheelPositions[2]}")
//        print("%% @OdometryLocalizer_REVNorm  LEFT=${robot.leftOdometry.getDistanceNormalized(INCH)}    RIGHT=${robot.rightOdometry.getDistanceNormalized(INCH)}    REAR=${robot.rearOdometry.getDistanceNormalized(INCH)}")
//        val packet = TelemetryPacket()
//        packet.put("left read", wheelPositions[0])
//        packet.put("right read", wheelPositions[1])
//        packet.put("rear read", wheelPositions[2])
//        FtcDashboard.getInstance().sendTelemetryPacket(packet)

        return wheelPositions
    }


}