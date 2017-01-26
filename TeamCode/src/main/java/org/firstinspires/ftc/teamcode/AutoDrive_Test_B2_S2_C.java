package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Samuel Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Blue Test: P2 Shoot2 Center Delay 10", group = "Vortex")

public class AutoDrive_Test_B2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        encoderDrive(DRIVE_SPEED,42,42,5.0);
        encoderDrive(DRIVE_SPEED,0,-8,5.0);
        //pushBeaconForward(true);
        //Step 1: Launch the balls.
        //encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        //turn90R();
        //encoderDrive(DRIVE_SPEED, -32.0, -32.0, 5.0);
        //turn90L();
        //launchBalls(2);
        //Step 2: Go to the center.
        //encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        //turn90L();
    }
}
