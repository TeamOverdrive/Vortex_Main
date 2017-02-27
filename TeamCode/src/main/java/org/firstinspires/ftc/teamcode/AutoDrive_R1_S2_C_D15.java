package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 1/7/2017.
 */

@Autonomous(name = "AutoDrive-Red: P1 S2 Ctr Delay 15", group = "Vortex")

public class AutoDrive_R1_S2_C_D15 extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        sleep(15*1000);
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED *.5, -22.0, -22.0, 5.0);
        launchBalls(2);
        //Step 2: Go to the center.
        encoderDrive(DRIVE_SPEED, -30.0, -30.0, 5.0);
        }
}