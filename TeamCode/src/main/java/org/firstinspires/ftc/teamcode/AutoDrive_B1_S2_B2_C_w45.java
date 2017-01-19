package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Turner on 1/18/2017.
 */

@Autonomous(name = "AutoDrive-Blue: P1 Shoot2 Beacon2 Center w45", group = "Vortex")

public class AutoDrive_B1_S2_B2_C_w45 extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        calibrateGyro();
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -18.0, -18.0, 5.0);
        encoderDrive(DRIVE_SPEED/2, -4.0, -4.0, 5.0);
        launchBalls(2);
        //Step 2: Get between the beacons.
        turn45R();
        encoderDrive(DRIVE_SPEED, -66.5, -66.5, 5.0);
        turn135R();
        //encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
        //turnGyroAbsR(90);
        //Needs adjustment
        //encoderDrive(DRIVE_SPEED, -26.0, -26.0, 3.0);
        //turnGyroAbsR(180);
        //Step 3: Hit the beacons.
        pushBeaconBackward(true);
    }
}