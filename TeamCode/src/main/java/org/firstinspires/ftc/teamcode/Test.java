package org.firstinspires.ftc.teamcode;

import org.openftc.revextensions2.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Func;

@TeleOp(name = "Test")
public class Test extends LinearOpMode {

    ExpansionHubEx expansionHub;

    @Override public void runOpMode() {

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");


        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("5v monitor", expansionHub.read5vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Voltage from the phone
            telemetry.addData("12v monitor", expansionHub.read12vMonitor(ExpansionHubEx.VoltageUnits.VOLTS)); //Battery voltage
            telemetry.update();
        }
    }
}