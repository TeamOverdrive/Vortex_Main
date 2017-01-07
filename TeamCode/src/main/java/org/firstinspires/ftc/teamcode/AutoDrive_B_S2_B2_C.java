package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Samuel Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Blue: P1 Shoot2 Beacon2 Center", group = "Vortex")

public class AutoDrive_B_S2_B2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -20.0, -20.0, 5.0);
        launchBalls(2);
        //Step 2: Get between the beacons.
        turn90R();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90L();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED, -12.0, -12.0, 3.0);
        turn90R();
        //Step 3: Hit the beacons.
        pushBeaconBackward(true);
        //Step 4: Reach the center
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
    }
}