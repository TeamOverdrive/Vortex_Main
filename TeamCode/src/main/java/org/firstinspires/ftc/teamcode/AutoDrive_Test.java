package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Samuel Turner
 * @version 2017.1.13
 */

@Autonomous(name="AutoDrive: Test", group="Vortex")

public class AutoDrive_Test extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0); //Max 5 seconds
        launchBalls(2); //Max 5 seconds
        turn90L();
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0);
        turn90R();
        encoderDrive(DRIVE_SPEED, 36.0, 36.0, 5.0);
        turn90L();
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0);
        turn90R();
        pushBeaconForward(true);
        turn90R();
        encoderDrive(DRIVE_SPEED, 52.0, 52.0, 10.0);
        //This is a comment.
    }
}
