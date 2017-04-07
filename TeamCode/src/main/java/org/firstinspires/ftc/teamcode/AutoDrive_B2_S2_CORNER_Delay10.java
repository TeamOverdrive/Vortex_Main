package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by S Turner on 2/5/2017.
 * Starting position is forward on the right side of the 5th floor tile
 */
@Autonomous(name="AutoDrive - Blue: P2 S2 Corner Delay 10", group="Vortex")

public class AutoDrive_B2_S2_CORNER_Delay10 extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //Step 1: Delay 11 seconds
        sleep(11 * 1000);
        //Step 2: Move to shooting position
        encoderDrive(DRIVE_SPEED * 0.5, -18.0, -18.0, 5.0);
        turn45R();
        encoderDrive(DRIVE_SPEED * 0.5, -28.0, -28.0, 5.0);
        //Step 3: Launch Balls
        launchBalls(2);
        //Step 4: Go to the center.
        turn45R();
        encoderDrive(DRIVE_SPEED, -56.0, -56.0, 5.0);
    }
}
