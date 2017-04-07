package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by Samuel Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Blue: P2 Shoot2 Center Defense", group = "Vortex")

public class AutoDrive_B2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();

        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED * 0.7, -6.0, -6.0, 3.0);
        turn45R();
        encoderDrive(DRIVE_SPEED * 0.5, -20.0, -20.0, 4.0);
        launchBalls(2);

        //Step 2: Move to defense location
        turn45L();
        turn90L();
        encoderDrive(1.0, -20.0, -20.0, 4.0);
        turn90R();
        encoderDrive(DRIVE_SPEED * 0.5, -6.0, -6.0, 3.0);

        //Step 3: Go to the center.
        turn90R();
        encoderDrive(DRIVE_SPEED, -28.0, -28.0, 5.0);
        encoderDrive(DRIVE_SPEED * 0.5, 8.0, 8.0, 3.0);

        /*
        //sleep(10*1000);
        Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED * 0.6, -24.0, -24.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED * 0.6, -28.0, -28.0, 5.0);
        turn90L();
        launchBalls(2);
        //Step 2: Go to the center.
        encoderDrive(DRIVE_SPEED, -18.0, -18.0, 5.0);
        turn90L();
        */
    }
}
