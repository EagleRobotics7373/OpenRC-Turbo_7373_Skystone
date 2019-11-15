package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.functions.Position;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;

@TeleOp(name = "Test", group = "Sensor")
public class Test extends LinearOpMode {

    BasicRobot robot;
    IMUController imuController;

    @Override public void runOpMode() {

        robot = new BasicRobot(hardwareMap);
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);

        // wait for the start button to be pressed
        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("left", robot.leftDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("right", robot.rightDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("front", robot.frontDistanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("heading", MathExtensionsKt.toDegrees(imuController.getHeading()));
            telemetry.update();

        }
    }

    private void doArmLift() {
        double currentValue = 2.0;
        double targetValue = 1.257;
        double PPower = 4.0;
        double originalRuntime = getRuntime();
        while ((currentValue=robot.intakePivotPotentiometer.getVoltage()) > targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<1) {
            robot.intakePivotMotor.setPower(-PPower * (targetValue-robot.intakePivotPotentiometer.getVoltage()));
        }
        robot.intakePivotMotor.setPower(0.12);
    }

    private void drive(double x, double y, double power) {
        robot.holonomic.runUsingEncoder(x, y, power);
        double originalRuntime = getRuntime();
        while (opModeIsActive() && robot.holonomic.motorsAreBusy() && getRuntime()-originalRuntime < 3) {

        }
    }

    /*public void imuPIRotate(double angle) {
        double currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        double Kp = .02; // Proportional Constant
        double Ki = .0007; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<3) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);

            if (currentValue < 0) {
                currentValue += 360;
            }

            et = targetValue - currentValue;

            if (Kp * et > .8) {
                proportionPower = .8;
            } else {
                proportionPower = Kp * et;
            }

            if (et < 45) {
                errorSum += et;
            }

            integralPower = Ki * errorSum;

            if (currentValue > 180) {
                power = (proportionPower + integralPower);
            } else {
                power = -(proportionPower + integralPower);
            }

            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            robot.holonomic.runWithoutEncoder(0,0,power*0.30);
            telemetry.update();
        }
        robot.holonomic.stop();
    }*/
    public void imuPIRotate(double angle) {
        double currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        double Kp = .02; // Proportional Constant
        double Ki = .0007; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        double originalRuntime = getRuntime();
        while (currentValue != targetValue && opModeIsActive() && (getRuntime()-originalRuntime)<3) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
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
            robot.holonomic.runWithoutEncoder(0,0,power*0.15);
            telemetry.update();
        }
        robot.holonomic.stop();
    }
}