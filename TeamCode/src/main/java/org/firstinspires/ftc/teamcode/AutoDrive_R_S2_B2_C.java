package org.firstinspires.ftc.teamcode;

/**
 * Created by Nathan on 1/6/2017.
 */
public class AutoDrive_R_S2_B2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        //DO NOT TOUCH THE ROBOT AT THIS TIME!
        calibrateGyro();
        //Wait for it...
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        waitForStart();
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -20.0, -20.0, 5.0);
        launchBalls(2);
        //Step 2: Get between the beacons.
        turn90L();
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90R();
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90L();
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -12.0, -12.0, 3.0);
        turn90R();
        telemetry.addData("0", "Heading %03d", gyroSensor.getHeading());
        telemetry.update();
        //Insert code here using the gyro
        //Step 3: Hit the beacons.
        pushBeaconForward(true);
        //Step 4: Reach the center
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
    }
}
