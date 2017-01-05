/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/* import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot; */

/**
 * This file uses the concept of driving a path based on time.
 * The code is structured as a LinearOpMode and assumes that
 * that encoders are NOT used on the wheels.
 *
 *   The desired path in this example is:
 *   - Drive forward for 2.5 seconds to get in position for the ball launcher
 *   - Turn on launcher for 2.0 seconds to launch first of two balls
 *   - Turn on ball intake for 4.0 seconds to load and launch second ball
 *   - Drive forward for 0.5 seconds to move the cap ball off the stand
 *
 * The code is written in a simple form with no optimizations and assumes that the
 * previous operation is stopped in the routine of the next step.
 *
 * Initial code was from the autonomous sample PushbotAutoDriveByTime_Linear.
 */

@Autonomous(name="AutoDrive: Enc Inside Shoot 2 Delay 10", group="Vortex")

public class AutoDrive_Encode_Sh2_IS_D_10 extends AutoSuper {
    @Override
    public void runOpMode() {
        super.runOpMode();
        //Removed code is now at the bottom of the class.
        waitForStart();

        // ********************************************************************************************************

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

// Step 0:  Wait for 10 seconds for 1st team to clear
        sleep(10000);

// Step 1:  Run with encoders to a position opposite the goal and pause
        encoderDrive(DRIVE_SPEED, 24, 24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(250);

// Step 2:  Run ball launcher for 3 seconds -- launch 1st ball
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.0)) {
            shooterMotor.setPower(-1.0);

            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Run ball intake for 3 seconds -- load 2nd ball
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            intakeMotor.setPower(1.0);

            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //  Step 4:  Stop launcher and intake to move to beacon locations
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);

        telemetry.addData("Path", "Complete");
        telemetry.update();

        /*
        // Step 5:  Turn Left
        encoderDrive(TURN_SPEED, -8.5, 8.5, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -4, -4, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(1000);     // pause for servos to move
*/
    }
}

/* Initialize the drive system variables.    */
// Define and Initialize Motors
        /*leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");
        shooterMotor = hardwareMap.dcMotor.get("shooter");
        intakeMotor = hardwareMap.dcMotor.get("intake");

        // Set the drive motor directions:
        // "REVERSE" the motor that runs backwards when connected directly to the battery.  Set to REVERSE if using AndyMark motors
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        shooterMotor.setPower(0);
        intakeMotor.setPower(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        // Set all motors to run with or without encoders.  Switch to use RUN_USING_ENCODERS when encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();*/

// Wait for the game to start (driver presses PLAY)

