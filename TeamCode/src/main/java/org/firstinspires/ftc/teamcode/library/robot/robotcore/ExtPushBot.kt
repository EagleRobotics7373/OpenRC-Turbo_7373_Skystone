package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.*

open class ExtPushBot(_hardwareMap: HardwareMap) : BaseRobot(_hardwareMap) {

    init {
        println("ExtPushBot being constructed!")
        RobotConstantsAccessor.load(
                DriveConstantsNew::class.java,
                OdometryConstantsPushbot::class.java
        )
    }

    // Drivetrain Variables

            // @JvmField val frontLeftMotor  : DcMotorEx FROM BaseRobot
            // @JvmField val backLeftMotor   : DcMotorEx FROM BaseRobot
            // @JvmField val frontRightMotor : DcMotorEx FROM BaseRobot
            // @JvmField val backRightMotor  : DcMotorEx FROM BaseRobot

    // Odometry Variables
     @JvmField val odometryLeft         : DcMotorEx             = hwInit("odometryLeft")
     @JvmField val odometryRight        : DcMotorEx             = hwInit("odometryRight")
     @JvmField val odometryRear         : DcMotorEx             = hwInit("odometryRear")

    override val leftOdometryModule     : OdometryModule?  = OdometryModule(odometryLeft)
    override val rightOdometryModule    : OdometryModule?  = OdometryModule(odometryRight)
    override val rearOdometryModule     : OdometryModule?  = OdometryModule(odometryRear)

    // IMU Variables
     @JvmField val imuControllerA       : IMUController         = IMUController(hardwareMap = hardwareMap, id= 'A')

    // Robot Systems Variables
            // val holonomic : Holonomic FROM BaseRobot

     override val holonomicRR             : HolonomicRR           = HolonomicRR(imuControllerA,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 if (RobotProvider.useTwoWheelOdometry)
                                                                                     TwoWheelOdometryLocalizer(odometryLeft, odometryRear, imuControllerA)
                                                                                else ThreeWheelOdometryLocalizer(odometryLeft, odometryRight, odometryRear))
}
