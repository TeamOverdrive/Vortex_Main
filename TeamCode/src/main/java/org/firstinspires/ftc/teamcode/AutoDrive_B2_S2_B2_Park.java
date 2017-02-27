package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * Created by S Turner on 2/5/2017.
 * Starting position is forward on the right side of the 5th floor tile
 */
@Autonomous(name="AutoDrive-Blue: P2 S2 B2 Park", group="Vortex")
@Disabled //Disabled program to drop from the phone list
public class AutoDrive_B2_S2_B2_Park extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        waitForStart();
        //Step 1: Move to shooting position
        encoderDrive(DRIVE_SPEED, -14.0, -14.0, 5.0);
        turn45R();
        encoderDrive(DRIVE_SPEED, -16.0, -16.0, 5.0);
        //Step 2: Launch Balls
        launchBalls(2);
        //Step 3: Go to the beacons
        encoderDrive(DRIVE_SPEED, -99.0, -99.0, 5.0);
        turn45L();
        //Step 4: Find beacons
        pushBeaconBackward(false);
        //Step 5: Park on ramp
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0);
    }
}
