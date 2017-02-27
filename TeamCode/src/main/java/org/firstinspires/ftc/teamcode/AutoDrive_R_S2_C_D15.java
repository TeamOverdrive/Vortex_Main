package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Nathan on 1/7/2017.
 */

@Autonomous(name = "AutoDrive-Red: P1 Shoot2 Center Delay 15", group = "Vortex")

public class AutoDrive_R_S2_C_D15 extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        sleep(15*1000);
        //Step 1: Launch the balls.
        //encoderDrive(DRIVE_SPEED, -22.0, -22.0, 5.0);
        //launchBalls(2);
        //Step 2: Go to the center.
        //encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        //turn90L();
        //Step 2: Move to shooting position
        encoderDrive(DRIVE_SPEED * 0.5, -14.0, -14.0, 5.0);
        turn45R();
        encoderDrive(DRIVE_SPEED * 0.5, -32.0, -32.0, 5.0);
        //Step 3: Launch Balls
        launchBalls(2);
        //Step 4: Go to the center.
        encoderDrive(DRIVE_SPEED, -14.0, -14.0, 5.0);
    }
}
