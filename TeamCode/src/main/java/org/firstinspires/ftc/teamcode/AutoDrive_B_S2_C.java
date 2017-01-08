package org.firstinspires.ftc.teamcode;

/**
 * Created by Nathan on 1/7/2017.
 */
public class AutoDrive_B_S2_C extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -22.0, -22.0, 5.0);
        launchBalls(2);
        //Step 2: Go to the center.
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90R();
    }
}
