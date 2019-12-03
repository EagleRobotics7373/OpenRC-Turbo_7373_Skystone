package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.library.functions.AllianceColor;
import org.firstinspires.ftc.teamcode.library.functions.ExtDirMusicPlayer;
import org.firstinspires.ftc.teamcode.library.functions.FieldSide;
import org.firstinspires.ftc.teamcode.library.functions.FunctionalExtensionsKt;
import org.firstinspires.ftc.teamcode.library.functions.Point3D;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VisionFactory;
import org.firstinspires.ftc.teamcode.library.vision.skystone.VuforiaController;
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.OpenCvContainer;
import org.firstinspires.ftc.teamcode.library.vision.skystone.opencv.PixelStatsPipeline;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Main")
public class AutonomousMeet4 extends LinearOpMode {
    BasicRobot robot;
    IMUController imuController;
    boolean goingRight = false;
    public static int TIMEMS = 1500;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
                Initialize main autonomous variables
         */
        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);
        AutoMenuControllerIterative menuController = new AutoMenuControllerIterative(telemetry);
        robot.intakeBlockGrabber.release();

        /*
                Operate telemetry menu
         */
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) {
                menuController.menu.previousItem();
                while (gamepad1.dpad_up && !isStopRequested()) ;
            } else if (gamepad1.dpad_down) {
                menuController.menu.nextItem();
                while (gamepad1.dpad_down && !isStopRequested()) ;
            } else if (gamepad1.dpad_left) {
                menuController.menu.iterateBackward();
                while (gamepad1.dpad_left && !isStopRequested()) ;
            } else if (gamepad1.dpad_right) {
                menuController.menu.iterateForward();
                while (gamepad1.dpad_right && !isStopRequested()) ;
            }
        }

        waitForStart();
        double startLeftDist = robot.leftDistanceSensor.getDistance(DistanceUnit.INCH);
        if (!isStopRequested()) {
        /*
                RoboSpotify
         */
            ExtDirMusicPlayer player = new ExtDirMusicPlayer(menuController.getMusicFile(), true);
            player.play();

            PIDFCoefficients pidf = ((DcMotorEx) robot.frontLeftMotor).getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("p", pidf.p);
            telemetry.addData("i", pidf.i);
            telemetry.addData("d", pidf.d);
            telemetry.addData("f", pidf.f);
            telemetry.update();
//            sleep(5000);
        /*
                Robot Actions
         */


            if (menuController.getParkOnly()) {
                sleep(menuController.getDelayBeforeParking() * 1000);
                drive(0, 12, 0.5);

            } else {
                if (menuController.getStartingPosition() == FieldSide.WAFFLE_SIDE) {
                    if (menuController.getAllianceColor() == AllianceColor.RED) {
                        if (menuController.getBuildingSiteSlide()) drive(24, 0, 0.8);
                        // Drive forward to clear the wall
                        //                drive(0, 5, 0.7);
                        //                sleep(500);
                        //                // Rotate 180 degrees using IMU PI controller
                        //                imuPIRotate(180);

                        // Find distance away from the wall (REMOVED)
//                        double distWall = 42 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                        // Drive to the foundation
                        drive(0, 29, 0.4);

                        sleep(250);

                        // Deploy the foundation grabber, grabbing the foundation
                        robot.foundationGrabbers.lock();
                        sleep(1000);

                        // Drive back to the wall
                        timeDrive(0, -0.5, 0, 2000);
//                        timeDrive(0, 0.5, 0, 2000);
                        // Release the foundation grabbers
                        robot.foundationGrabbers.unlock();
                        sleep(500);


                        if (menuController.getParkAfterTask()) {
//                            // Drive toward the alliance bridge to start moving around the foundation
//                            drive(30, 0, 0.2);
//                            // Drive parallel to the bridges to move to the other side of the foundation
//                            drive(0, -18, 0.2);
//                            drive(-10, 0, 0.2);
//
//                            drive(60, 14, .7);
//
//                            imuPIRotate(90);
//                            telemetry.addData("heading", MathExtensionsKt.toDegrees(imuController.getHeading()));
//                            telemetry.update();
//                            sleep(2000);

//                            // Park under the bridge
//                            drive(23, 0, 0.6);
//                            if (menuController.getParkNearDS()) timeDrive(0,0.3, 0, 1000);
//                            else timeDrive(0, -0.3, 0, 500);

                            drive(-35, 0, 0.2);
                            // Drive parallel to the bridges to move to the other side of the foundation
                            if (menuController.getFoundationRedundancy()) {
                                drive(0, 17, 0.2);
                                //push foundation
                                drive(20, 0, 0.2);
                                sleep(1000);
                                // drive back
                                drive(-26, 0, 0.2);
                            } else {
                                drive(-16, 0, 0.5);
                                drive(0, 8, 0.4);
                            }
                            if (menuController.getParkNearDS()) drive(0, -24, 0.2);
                            else {
                                timeDrive(0, 0.4, 0, 500);
                                sleep(500);
                                robot.holonomic.stop();
                            }
                        }

                    } else { // Blue side waffle
                        double startingRuntime = getRuntime();
                        if (menuController.getBuildingSiteSlide()) drive(-24, 0, 0.7);
                        // Drive forward to clear the wall
                        //                drive(0, 5, 0.4);
                        //                sleep(500);
                        // Find distance away from the wall
                        //                double distWall = 44 - robot.distanceSensor_side.getDistance(DistanceUnit.INCH);

                        // Drive to the foundation
                        drive(0, 29, 0.4);
                        sleep(250);
                        telemetry.addData("blab blab blab", "wow taco");
                        telemetry.update();
                        // Deploy the foundation grabber, grabbing the foundation
                        robot.foundationGrabbers.lock();
                        sleep(2000);

                        // Drive back to the wall
//                        drive(0, 36, 0.2);
                        timeDrive(0, -0.5, 0, 2000);

                        // Release the foundation grabbers
                        robot.foundationGrabbers.unlock();
                        sleep(500);
                        if (menuController.getParkAfterTask()) {
                            // Drive toward the alliance bridge to start moving around the foundation
                            drive(35, 0, 0.2);
                            // Drive parallel to the bridges to move to the other side of the foundation
                            if (menuController.getFoundationRedundancy()) {
                                drive(0, 17, 0.2);
                                //push foundation
                                drive(-20, 0, 0.2);
                                sleep(1000);
                                // drive back
                                drive(26, 0, 0.2);
                            } else {
                                drive(12, 0, 0.5);
                                while (getRuntime() - startingRuntime < menuController.getDelayBeforeParking());
                            }
                            if (menuController.getParkNearDS()) drive(0, -24, 0.2);
                            else {
                                timeDrive(0, 0.4, 0, 500);
                                sleep(500);
                                robot.holonomic.stop();
                            }
                        }

                    }
                }
                else { // FIELD POSITION IS LOADING ZONE!!!
                    OpenCvContainer<PixelStatsPipeline> container = VisionFactory.createOpenCv(
                            VisionFactory.CameraType.WEBCAM,
                            hardwareMap,
                            new PixelStatsPipeline(menuController.getVisionDetector()));
                    sleep(2000);

                    Position skystonePosition = container.getPipeline().getSkystonePos();


                    if (menuController.getAllianceColor() == AllianceColor.RED) {
                        if (skystonePosition == Position.LEFT) {
                            skystonePosition = Position.CENTER;
                        }
                    }

                    drive(0, 29.5, 0.4);


                }
            }
        /*
                End of OpMode - close resources
         */
            player.stop();
        }
    }

    public void imuPIRotate(double angle) {
        double currentValue = FunctionalExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        double Kp = .02; // Proportional Constant
        double Ki = .0007; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime() - originalRuntime) < 4) {
            currentValue = FunctionalExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);

            if (currentValue < 0) {
                et = -(Math.abs(targetValue) - Math.abs(currentValue));
            } else {
                et = targetValue - currentValue;
            }


            if (Kp * et > .8) {
                proportionPower = .8;
            } else {
                proportionPower = Kp * et;
            }

            if (Math.abs(et) < 45) {
                errorSum += et;
            }

            integralPower = Ki * errorSum;

            power = -(proportionPower + integralPower);
            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            robot.holonomic.runWithoutEncoder(0, 0, power * 0.30);
            telemetry.update();
        }
        robot.holonomic.stop();
    }

    private void doArmLift(double target) {
        double currentValue = 3.0;
        double targetValue = target;
        double PPower = 3.5;
        double originalRuntime = getRuntime();
        while ((currentValue = robot.intakePivotPotentiometer.getVoltage()) > targetValue && opModeIsActive() && (getRuntime() - originalRuntime) < 1) {
            robot.intakePivotMotor.setPower(-PPower * (targetValue - robot.intakePivotPotentiometer.getVoltage()));
            telemetry.addData("Current", currentValue);
            telemetry.update();
        }
        robot.intakePivotMotor.setPower(0.07);
    }

    private void timeDrive(double x, double y, double z, long timeMs) {
        robot.holonomic.runWithoutEncoder(x, y, z);
        sleep(timeMs);
        robot.holonomic.stop();
    }

    private void turn(double degrees, double power) {
        robot.holonomic.turnUsingEncoder(180, 0.5);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime() - originalRuntime < 2)
            ;
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime() - originalRuntime < 3) {

        }
    }

    private double findAvgXDist(double xTarget) {
        double WHEEL_DIAMETER = 4.0;
        double WHEEL_CIRCUMFERENCE = (WHEEL_DIAMETER * Math.PI);
        double TICKS_PER_REVOLUTION = 450.0;
        double TICKS_PER_INCH /* 134.4*/ = TICKS_PER_REVOLUTION / WHEEL_CIRCUMFERENCE;

        double r;
        double theta;
        double axisConversionAngle = Math.PI / 4;
        double xPrime;
        double yPrime;
        double xPower;
        double yPower;

        double LFDistanceIN;
        double LRDistanceIN;
        double RRDistanceIN;
        double RFDistanceIN;
        double LFPower;
        double LRPower;
        double RRPower;
        double RFPower;

        // calculate r
        r = Math.sqrt(Math.pow(xTarget, 2) + Math.pow(0, 2));
        // calculate theta
        if (xTarget == 0) xTarget = 0.00001;
        theta = Math.atan(0 / xTarget);
        if (xTarget < 0) theta += Math.PI;
        // calculate x and y prime
        xPrime = r * Math.cos(theta - axisConversionAngle);
        yPrime = r * Math.sin(theta - axisConversionAngle);

        double result = ((Math.abs(xPrime*TICKS_PER_INCH) + Math.abs(yPrime*TICKS_PER_INCH)) / 2);
        if (xTarget < 0) {
            goingRight = false;
            result *= -1;
        } else {
            goingRight = true;
        }
        return result;
    }

    private double avgReadEncoderDistances() {
//        LFDistanceIN = xPrime;
//        LRDistanceIN = yPrime;
//        RRDistanceIN = -xPrime;
//        RFDistanceIN = -yPrime;
        double flmcp =  Math.abs(robot.frontLeftMotor.getCurrentPosition());
        double brmcp =  Math.abs(robot.backRightMotor.getCurrentPosition());
        double frmcp =  Math.abs(robot.frontRightMotor.getCurrentPosition());
        double blmcp =  Math.abs(robot.backLeftMotor.getCurrentPosition());

        telemetry.addData("flm cp", flmcp);
        telemetry.addData("brm cp", brmcp);
        telemetry.addData("frm cp", frmcp);
        telemetry.addData("blm cp", blmcp);
        double sum = flmcp + brmcp + frmcp + blmcp;
        double avg = sum/4;

        telemetry.addData("sum", sum);
        telemetry.addData("avg", avg);

        return goingRight?avg:-avg;
//        return robot.frontLeftMotor.getCurrentPosition();

    }


}