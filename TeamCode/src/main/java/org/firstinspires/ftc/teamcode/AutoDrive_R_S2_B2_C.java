package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Samuel Turner on 1/6/2017.
 */
@Autonomous(name = "AutoDrive-Red: P1 Shoot2 Beacon2 Center", group = "Vortex")

public class AutoDrive_R_S2_B2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        calibrateGyro();
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -22.0, -22.0, 5.0);
        launchBalls(2);
        //Step 2: Get between the beacons.
        turn90L();
        encoderDrive(DRIVE_SPEED, -18.0, -18.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
        turn90L();
        //Needs adjustment
        encoderDrive(DRIVE_SPEED, -20.0, -20.0, 3.0);
        turn90R();
        //Step 3: Hit the beacons.
        pushBeaconForward(true);
        //Step 4: Reach the center
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
    }
}
