package org.firstinspires.ftc.teamcode.library.robot.robotcore

import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.TwoWheelOdometryLocalizer
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.FoundationGrabbers
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.AutoBlockIntake
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.CapstonePlacer
import org.firstinspires.ftc.teamcode.library.robot.systems.wrappedservos.IntakeBlockGrabber
import org.openftc.revextensions2.ExpansionHubEx
import org.openftc.revextensions2.ExpansionHubMotor

open class PushBot(protected val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor          : ExpansionHubMotor     = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor           : ExpansionHubMotor     = hwInit("backLeftMotor")
     @JvmField val frontRightMotor         : ExpansionHubMotor     = hwInit("frontRightMotor")
     @JvmField val backRightMotor          : ExpansionHubMotor     = hwInit("backRightMotor")

     @OdometryDevice("left") @JvmField val odometryLeft          : ExpansionHubMotor     = hwInit("odometryLeft")
     @JvmField val odometryRight         : ExpansionHubMotor     = hwInit("odometryRight")
     @JvmField val odometryRear             : ExpansionHubMotor     = hwInit("odometryRear")


     @JvmField val odometryRearAsMotor     : ExpansionHubMotor     = hwInit("odometryRearAsMotor")

    // Servo/PWM Variables
    @JvmField val foundationGrabFrontLeft : Servo                 = hwInit("foundationGrabFrontLeft")
    @JvmField val foundationGrabFrontRight: Servo                 = hwInit("foundationGrabFrontRight")
    @JvmField val foundationGrabSideFront : Servo                 = hwInit("foundationGrabSideFront")
    @JvmField val foundationGrabSideRear  : Servo                 = hwInit("foundationGrabSideRear")

    @JvmField val autoBlockGrabFront      : Servo                 = hwInit("autoBlockGrabFront")
    @JvmField val autoBlockPivotFront     : Servo                 = hwInit("autoBlockPivotFront")
    @JvmField val autoBlockGrabRear       : Servo                 = hwInit("autoBlockGrabRear")
    @JvmField val autoBlockPivotRear      : Servo                 = hwInit("autoBlockPivotRear")

    @JvmField val intakeBlockGrabberServo : Servo                 = hwInit("intakeBlockGrabber")
    @JvmField val capstonePlacerAsServo   : Servo                 = hwInit("capstonePlacement")
    @JvmField val blinkin                 : RevBlinkinLedDriver   = hwInit("blinkin")

    // Expansion Hub Variables
       private val expansionHubA           : ExpansionHubEx        = hwInit("Expansion Hub A")
       private val expansionHubB           : ExpansionHubEx        = hwInit("Expansion Hub B")

    // IMU Variables
     @JvmField val imuControllerA          : IMUController         = IMUController(hardwareMap = hardwareMap, id= "imuA")

//     Robot Systems Variables
     @JvmField val foundationGrabbersFront : FoundationGrabbers = FoundationGrabbers(foundationGrabFrontLeft, 0.00, 0.49, true,
        foundationGrabFrontRight, 0.80, 0.25, false)

     @JvmField val foundationGrabbersSide   : FoundationGrabbers = FoundationGrabbers(foundationGrabSideFront, 0.43, 1.00, true,
             foundationGrabSideRear, 0.89, 0.30, true)

     @JvmField val intakeBlockGrabber      : IntakeBlockGrabber = IntakeBlockGrabber(intakeBlockGrabberServo, 0.00, 0.30, 1.00)

     @JvmField val holonomic               : Holonomic = Holonomic(frontLeftMotor as DcMotor, backLeftMotor as DcMotor, frontRightMotor as DcMotor, backRightMotor as DcMotor)
     @JvmField val holonomicRR             : HolonomicRR           = HolonomicRR(hardwareMap, imuControllerA,
                                                                                 frontLeftMotor, backLeftMotor, backRightMotor, frontRightMotor,
                                                                                 TwoWheelOdometryLocalizer(odometryLeft, odometryRearAsMotor, expansionHubA, imuControllerA))

     @JvmField val autoBlockIntakeFront    : AutoBlockIntake = AutoBlockIntake(
             pivotServo = autoBlockPivotFront,  pivot18 = 0.92, pivotMid = 0.79, pivotVertical = 0.89, pivotPickup = 0.52,
             grabberServo = autoBlockGrabFront, grab18  = 0.99, grabUp = 0.55, grabMid = 0.62, grabPickup = 0.95)
     @JvmField val autoBlockIntakeRear     : AutoBlockIntake = AutoBlockIntake(
            pivotServo = autoBlockPivotRear,  pivot18 = 0.20, pivotMid = 0.48, pivotVertical = 0.36, pivotPickup = 0.71,
            grabberServo = autoBlockGrabRear, grab18  = 0.30, grabUp = 0.55, grabMid = 0.80, grabPickup = 0.99)

     @JvmField val capstonePlacer = CapstonePlacer(capstonePlacerAsServo, pos18 = 0.35, posInside = 0.15, posDeploy = 0.75)

     @JvmField val odometryModuleLeft = OdometryModule(odometryLeft as DcMotor)
     @JvmField val odometryModuleRear = OdometryModule(odometryRearAsMotor as DcMotor)


    protected inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)
}
