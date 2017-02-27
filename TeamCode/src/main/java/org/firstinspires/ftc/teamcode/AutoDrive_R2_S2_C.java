package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 1/6/2017.
 */

@Autonomous(name = "AutoDrive-Red: P2 Shoot2 Center Delay 10", group = "Vortex")

public class AutoDrive_R2_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        sleep(10*1000);
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED * 0.5, -14.0, -14.0, 5.0);
        turn45L();
        encoderDrive(DRIVE_SPEED * 0.5, -28.0, -28.0, 5.0);
        launchBalls(2);
        //Step 2: Go to the center.
        encoderDrive(DRIVE_SPEED, -18.0, -18.0, 5.0);
    }
}
