package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 1/27/2017.
 */

@Autonomous(name = "AutoDrive-Blue: P1 B2 S2 Ctr", group = "Vortex")

public class AutoDrive_B1_B2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        calibrateGyro();
        waitForStart();

        //Step 1: Drive to side wall to position for beacon pushing.
        gyroDrive(DRIVE_SPEED, 52.0, 0.0);
        turn45L();
        encoderDrive(DRIVE_SPEED, 36.0, 36.0, 5.0);

        //Step 2:  Run beacon push routine
        pushBeaconBackward(false);

        //Step 3:  Position for and launch the balls
        turn90R();
        encoderDrive(DRIVE_SPEED, -12.0, -12.0, 5.0);
        launchBalls(2);

        //Step 4:  Drive to center and stop
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
    }
}