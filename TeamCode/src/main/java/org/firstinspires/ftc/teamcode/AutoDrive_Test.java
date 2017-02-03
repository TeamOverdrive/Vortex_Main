package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * @author Samuel Turner
 * @version 2017.1.13
 */

@Autonomous(name="AutoDrive: Test", group="Vortex")

public class AutoDrive_Test extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        calibrateGyro();
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

        /*leftMotor.setPower(0.5-((8.0 - SONIC_RANGE)*0.05));
        rightMotor.setPower(0.5+((8.0 - SONIC_RANGE)*0.05));
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Distance", ultrasonicSensor.getDistance(DistanceUnit.CM));
            String out = Double.toString(ultrasonicSensor.getDistance(DistanceUnit.CM));
            SONIC_RANGE = ultrasonicSensor.getDistance(DistanceUnit.CM);
            RobotLog.d(out);
            telemetry.update();
            }

    }*/
        while (opModeIsActive()){
            //gyroDrive(DRIVE_SPEED, 42.0, 135.0);
            encoderDrive(DRIVE_SPEED/2, 15.2, 1.0, 3.0);
            encoderDrive(DRIVE_SPEED/2, 1.0, 15.2, 3.0);
            sleep(5000);

    /*leftMotor.setPower((DRIVE_SPEED/4) * 1);
    rightMotor.setPower((DRIVE_SPEED/4) * 1);
    runtime.reset();
    while(opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 5 ) {
        telemetry.addData("Light Level", opticalSensor.getLightDetected());
        String out = Double.toString(opticalSensor.getLightDetected());
        RobotLog.d(out);
        telemetry.update();
    }
    leftMotor.setPower(0.0);
    rightMotor.setPower(0.0);
    //return runtime.seconds() < 3;
}
*/
        /* while(opModeIsActive() && ultrasonicSensor.getDistance(DistanceUnit.CM) >= 8) {
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.5);
            sleep(500);}
        while(opModeIsActive() && ultrasonicSensor.getDistance(DistanceUnit.CM) <= 6); {
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.3);
            sleep(500);}
        */
            //pushBeaconBackward(true);
        }
    }
}