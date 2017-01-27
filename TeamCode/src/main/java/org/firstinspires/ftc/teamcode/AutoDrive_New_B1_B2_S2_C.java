package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 1/27/2017.
 */

@Autonomous(name = "AutoDrive-Blue: New P1 Beacon2 Shoot2 Center", group = "Vortex")

public class AutoDrive_New_B1_B2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        calibrateGyro();
        waitForStart();

        //Step 1: Drive to side wall to position for beacon pushing.
        gyroTurn(TURN_SPEED, -45.0);
        gyroDrive(DRIVE_SPEED, -41.0, 0.0);
        gyroTurn(TURN_SPEED, 45.0);

        //Step 2:  Run beacon push routine
        pushBeaconBackward(true);

        //Step 3:  Position for and launch the balls
        gyroTurn(TURN_SPEED, 90.0);
        gyroDrive(DRIVE_SPEED, 12.0, 0.0);
        launchBalls(2);

        //Step 4:  Drive to center and stop
        gyroDrive(DRIVE_SPEED, 24.0, 0.0);
    }
}