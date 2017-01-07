package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * @author Samuel Turner
 * @version 2017.1.13
 */

@Autonomous(name="AutoDrive: Test", group="Vortex")

public class AutoDrive_Test extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //calibrateGyro();
        waitForStart();/*
        boolean light = false;
        colorSensor1.enableLed(light);
        colorSensor2.enableLed(light);
        while(opModeIsActive()) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            telemetry.addData("Color 1 Red", colorSensor1.red());
            telemetry.addData("Color 1 Blue", colorSensor1.blue());
            telemetry.addData("Color 2 Red", colorSensor2.red());
            telemetry.addData("Color 2 Blue", colorSensor2.blue());
            String out = "Sensor 1 red: " + Double.toString(colorSensor1.red());
            RobotLog.d(out);
            out = "Sensor 1 blue: " + Double.toString(colorSensor1.blue());
            RobotLog.d(out);
            out = "Sensor 2 red: " + Double.toString(colorSensor2.red());
            RobotLog.d(out);
            out = "Sensor 2 blue: " + Double.toString(colorSensor2.blue());
            RobotLog.d(out);
            telemetry.update();
        }*/
        encoderDrive(DRIVE_SPEED*0.8, -20.0, -20.0, 5.0);
        launchBalls(2);
        turn90L();
        encoderDrive(DRIVE_SPEED, -16.0, -16.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
        turn90L();
        encoderDrive(DRIVE_SPEED*0.8, -21.0, -21.0, 5.0);
        turn90R();
        pushBeaconBackward(true);
    }
}
