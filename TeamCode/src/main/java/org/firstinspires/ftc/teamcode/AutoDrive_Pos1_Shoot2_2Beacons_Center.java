package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * @author Samuel Turner
 * @version 2017.1.3
 */

@Autonomous(name = "AutoDrive: Pos1-Shoot 2-2 Beacons-End Center", group = "Vortex")

public class AutoDrive_Pos1_Shoot2_2Beacons_Center extends AutoSuper {
    @Override
    public void runOpMode() {
        //Initialize everything.
        super.runOpMode();
        //Wait for it...
        waitForStart();
        //Step 1: Launch the balls.
        encoderDrive(DRIVE_SPEED, -20.0, -20.0, 5.0);
        launchBalls(2);
        //Step 2: Get between the beacons.
        turn90R();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90L();
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED, -12.0, -12.0, 3.0);
        turn90L();
        //Step 3: Hit the beacons.
        pushBeaconForward(false);
        //Step 4: Reach the center
        turn90R();
        encoderDrive(DRIVE_SPEED, -36.0, -36.0, 5.0);
        turn90L();
        encoderDrive(DRIVE_SPEED, -12.0, -12.0, 4.0);
    }
}
