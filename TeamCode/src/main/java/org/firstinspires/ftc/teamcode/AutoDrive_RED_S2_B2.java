package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * @author Samuel Turner
 * @version 2017.1.13
 */

@Autonomous(name="AutoDrive: Inside 2 Beacon (RED)", group="Vortex")

public class AutoDrive_RED_S2_B2 extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        calibrateGyro();
        waitForStart();

   // Red Side Test Autonomous
        //Step 1: Drive up and shoot the balls
        encoderDrive(1.0, -15.0, -15.0, 5.0);
        encoderDrive(0.5, -5.0, -5.0, 2.0);
        launchBalls(2);
        //encoderDrive(DRIVE_SPEED * 0.5, -4.0, -4.0, 2.0);

        //Step 2: Position to push the beacons
        //turnGyroAbs(325);
        turn35L();
        encoderDrive(0.8, -60.0, -60.0, 5.0);
        encoderDrive(DRIVE_SPEED * 0.5, -8.0, -8.0, 3.0);
        turn35R();
        turnGyroAbs(0);

        //Step 3:  Run beacon push routine
        pushBeaconForward(true);

        //Step 4: Go to the ramp
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(1.0);
        rightMotor.setPower(0.4);
        sleep(1500);
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
        /*
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
        }
        */

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

    }
        while (opModeIsActive()) {
            if (ultrasonicSensor.getDistance(DistanceUnit.CM) <= 10.0) {
                distanceFlag.setPosition(0.0);
            } else distanceFlag.setPosition(0.5);
            sleep(100);

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

            while (opModeIsActive()) {
                if (ultrasonicSensor.getDistance(DistanceUnit.CM) >= 17) {
                    leftMotor.setPower(0.9);
                    rightMotor.setPower(0.1);
                } else if (ultrasonicSensor.getDistance(DistanceUnit.CM) >= 9) {
                    leftMotor.setPower(0.5 + ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05));
                    rightMotor.setPower(0.5 - ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05));
                } else if (ultrasonicSensor.getDistance(DistanceUnit.CM) <= 7) {
                    leftMotor.setPower(0.5 + ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05));
                    rightMotor.setPower(0.5 - ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05));
                } else {
                    leftMotor.setPower(0.5);
                    rightMotor.setPower(0.5);
                }
                sleep(100);

                telemetry.addData("Ultrasonic", ultrasonicSensor.getDistance(DistanceUnit.CM));
                telemetry.update();
                //pushBeaconBackward(true);
            }
        }
    }
}
*/
/*

    while(opModeIsActive() {
        if (ultrasonicSensor.getDistance(DistanceUnit.CM) <= 10.0) {
            distanceFlag.setPosition(0.0);
        } else distanceFlag.setPosition(0.5);
        sleep(100);

        final double MAX_CHANGE = 0.25; // Max expected cm change per cycle. No idea what to expect here. Tune up or down. This sould be somewhere a bit less than how far robot can drive in 1/10 second.
        double prevDist;
        double curDist;
        double curChange;
        double targetChange = 0.0;
        double desiredChange;
        double targetDist;
        double deltaFromTarget;
        int dir = 1;

        targetDist = 10.0;
        prevDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
        sleep(100);
        while (opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 8) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            String out = Double.toString(opticalSensor.getLightDetected());
            RobotLog.d(out);

            // First calculate how fast we want to be moving towards target
            curDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
            deltaFromTarget = curDist - targetDist; // positive if currently greater than target
            if (deltaFromTarget > 10.0) { // Far from target
                desiredChange = -1 * MAX_CHANGE;
            } else if (deltaFromTarget > 1.0) { // Getting close to target
                desiredChange = -1 * (deltaFromTarget / 10.0) * MAX_CHANGE; // between zero and negative max based on how far from target
            } else if (deltaFromTarget < -1.0) { // Went past target
                desiredChange = -1 * (deltaFromTarget / 10.0) * MAX_CHANGE; // between zero and positive max based on how far from target
            } else { // On target
                desiredChange = 0;
            }

            // Next adjust direction if not getting there fast enough
            curChange = curDist - prevDist; // Change since last check. Negative if distance is getting smaller
            if (curChange > targetChange + 0.1 * MAX_CHANGE) { // going further left than desired rate of change
                leftMotor.setPower((0.9) * dir);
                rightMotor.setPower((0.1) * dir);
                sleep(100); // Turn a little more
            } else if (curChange < targetChange - 0.1 * MAX_CHANGE) { // going further right than desired rate of change
                leftMotor.setPower((0.1) * dir);
                rightMotor.setPower((0.9) * dir);
                sleep(100); // Turn a little more
            } // else we're close to disired rate of change

            // Drive straight a little bit
            leftMotor.setPower((DRIVE_SPEED / 2) * dir);
            rightMotor.setPower((DRIVE_SPEED / 2) * dir);
            sleep(100); // Time to go straight
            telemetry.update();
            prevDist = curDist;
        }
    }
}
*/
