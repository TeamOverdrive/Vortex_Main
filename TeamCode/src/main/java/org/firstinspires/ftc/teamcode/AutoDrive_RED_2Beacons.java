package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * @author Samuel Turner
 * @version 2017.1.13
 */

@Autonomous(name="AutoDrive: 2 Beacon (RED)", group="Vortex")

public class AutoDrive_RED_2Beacons extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        calibrateGyro();
        waitForStart();

   // Red Side Test Autonomous
        //Setup is designed to start 48" from the corner

        //Step 1: Drive to space between white lines
        encoderDrive(0.8, -6.0, -6.0, 5.0);
        turn30L();
        encoderDrive(0.9, -55.0, -55.0, 5.0);
        encoderDrive(0.5, -5.0, -5.0, 2.0);
        turn30R();

        //Step 2: Run beacon push routine
        pushBeaconForward(true);

        //Step 3:  Position for and launch the balls
        turn90R_RED();
        encoderDrive(DRIVE_SPEED, -16.0, -16.0, 5.0);
        launchBalls(2);

        //Step 4:  Drive to center and stop
        encoderDrive(DRIVE_SPEED, -20.0, -20.0, 5.0);

    }
}
