package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "testTeleOp")
public class templateTeleOp extends LinearOpMode {

    // Importing handlers
    templateDrive drive = new templateDrive(hardwareMap);

    // Drive Vars
    boolean driveAllowed = true;

    double xMove;
    double yMove;
    double rX ;
    double d_multiplier = 0.5;


    // Timer
    ElapsedTime tm1;


    public void runOpMode() {

        waitForStart();

        tm1 = new ElapsedTime();

        while(opModeIsActive()) {

            xMove = gamepad1.left_stick_x;
            yMove = -gamepad1.left_stick_y;
            rX = gamepad1.right_stick_x;

            if (driveAllowed) {
                drive.run(xMove, yMove, rX, d_multiplier);
            }
        }
    }
}