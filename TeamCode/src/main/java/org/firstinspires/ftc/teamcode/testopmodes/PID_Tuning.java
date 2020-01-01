package org.firstinspires.ftc.teamcode.testopmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.library.functions.MathExtensionsKt;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.BasicRobot;
import org.firstinspires.ftc.teamcode.library.robot.robotcore.IMUController;
import org.firstinspires.ftc.teamcode.library.robot.systems.Holonomic;
import org.openftc.revextensions2.ExpansionHubEx;

@Config
class RobotConstants {
    public static double Kp = .0051;
    public static double Ki = .0005;
    public static double angle = 180;


    //90 degrees
    //0.007
    //0.0006

    // 180 degrees 13.92V
    //.0047
    //.0004
}

@TeleOp (name = "PID Tuning")
public class PID_Tuning extends LinearOpMode {

    Holonomic holonomic;

    ExpansionHubEx expansionHub;
    IMUController imuController;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    RobotConstants constants = new RobotConstants();
    double Kp = constants.Kp;
    double Ki = constants.Ki;
    double originalRuntime;
    double globalCV;
    double globalAngle;

    @Override public void runOpMode() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        holonomic = new Holonomic(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        imuController = new IMUController(hardwareMap, AxesOrder.ZYX);

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 3");


//        hardwareMap.voltageSensor.get("Expansion Hub Portal 1").getVoltage();

        waitForStart();

        originalRuntime = getRuntime();
        imuPIRotate(constants.angle);
        while (opModeIsActive()) {
            packet.put("angle", globalAngle);
            packet.put("current Value", globalCV);
            packet.put("voltage", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void imuPIRotate(double angle) {
        double currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
        double targetValue = currentValue + angle;

        if (angle == 180) {
            double Kp = .0051;
            double Ki = .0006;
        }
//        double Kp = .02; // Proportional Constant
//        double Ki = .0000; // Integral Constant
        double et; // Error
        double proportionPower;
        double integralPower;
        double power;
        double errorSum = 0;
        String lastError = "below";
        String etValue;
        while (Math.abs(targetValue - currentValue) > .1 && opModeIsActive()) {
            currentValue = MathExtensionsKt.toDegrees(imuController.getHeading());
            telemetry.addData("Current value", currentValue);
            telemetry.addData("Target value", targetValue);

            if (currentValue < 0) {
                currentValue += 360;
            }

            et = targetValue - currentValue;
            proportionPower = Kp * et;

            if (Math.abs(et) < 30) {
                errorSum += et;
            }

            if (currentValue > angle) {
                etValue = "above";
            } else if (currentValue < angle) {
                etValue = "below";
            } else {
                etValue = "target";
            }

            if (!(etValue.equals(lastError))) {
                errorSum = 0;
            }

            lastError = etValue;

            integralPower = Ki * errorSum;

            power = (proportionPower + integralPower);
            telemetry.addData("et", et);
            telemetry.addData("propPw", proportionPower);
            telemetry.addData("intPw", integralPower);
            telemetry.addData("errorsum", errorSum);
            telemetry.addData("Power", power);
            holonomic.runWithoutEncoder(0, 0, power);
            telemetry.update();

            packet.put("current value", currentValue);
            packet.put("target value", angle);
            packet.put("voltage", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS));
            dashboard.sendTelemetryPacket(packet);
        }
        holonomic.stop();
        globalCV = currentValue;
        globalAngle = angle;
    }
}
