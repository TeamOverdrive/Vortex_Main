package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Samuel Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Blue: New P1 Beacon2 Shoot2 Center", group = "Vortex")

public class AutoDrive_New_B1_B2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        calibrateGyro();
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, 41.0, 41.0, 5.0);
        //launchBalls(2);
        //Step 2: Get between the beacons.
        turnGyroAbsR(45);
        launchBalls(2);
        //encoderDrive(DRIVE_SPEED, -18.0, -18.0, 5.0);
        //turn90L();
        //encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
        //turnGyroAbsR(90);
        //Needs adjustment
        //encoderDrive(DRIVE_SPEED, -26.0, -26.0, 3.0);
        //turnGyroAbsR(180);
        //Step 3: Hit the beacons.
        pushBeaconBackward(true);
    }
}