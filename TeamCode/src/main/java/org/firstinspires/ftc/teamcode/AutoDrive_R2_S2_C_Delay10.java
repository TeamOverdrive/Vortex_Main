package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 2/5/2017.
 * Starting position is forward on the right side of the 5th floor tile
 */
@Autonomous(name="AutoDrive - Red: P2 S2 Ctr Delay 10", group="Vortex")

public class AutoDrive_R2_S2_C_Delay10 extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //Step 1: Delay 10 seconds
        sleep(10 * 1000);
        //Step 2: Move to shooting position
        encoderDrive(DRIVE_SPEED, -14.0, -14.0, 5.0);
        turn45L();
        encoderDrive(DRIVE_SPEED, -16.0, -16.0, 5.0);
        //Step 3: Launch Balls
        launchBalls(2);
        //Step 4: Go to the center.
        encoderDrive(DRIVE_SPEED, -27.0, -27.0, 5.0);
    }
}
