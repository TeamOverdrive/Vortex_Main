package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Red: P2 Shoot2 Center Defense", group = "Vortex")

public class AutoDrive_R2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //sleep(10*1000);

        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -6.0, -6.0, 3.0);
        turn45L();
        encoderDrive(DRIVE_SPEED * 0.5, -32.0, -32.0, 4.0);
        launchBalls(2);

        //Step 2: Move to defense location
        turn35R();
        turn90R();
        encoderDrive(1.0, -20.0, -20.0, 4.0);
        turn90L();
        encoderDrive(DRIVE_SPEED * 0.5, -6.0, -6.0, 3.0);

        //Step 3: Go to the center.
        turn90L();
        encoderDrive(DRIVE_SPEED, -28.0, -28.0, 5.0);
        encoderDrive(DRIVE_SPEED * 0.5, 8.0, 8.0, 3.0);
    }
}
