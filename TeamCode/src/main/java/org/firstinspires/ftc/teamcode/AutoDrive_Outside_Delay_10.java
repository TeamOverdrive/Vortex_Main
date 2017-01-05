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

@Autonomous(name="AutoDrive: Outside Time Delay 10S", group="Vortex")

public class AutoDrive_Outside_Delay_10 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor shooterMotor;
    public DcMotor intakeMotor;

    /*    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware  */
    private ElapsedTime runtime;

    {
        runtime = new ElapsedTime();
    }

    // FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.
    static final double FORWARD_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {
        /* Initialize the drive system variables.    */
        // Define and Initialize Motors
        leftMotor = hardwareMap.dcMotor.get("left_drive");
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

        // Set all motors to run without encoders.  Switch to use RUN_USING_ENCODERS when encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // ********************************************************************************************************

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 0:  Wait 10 seconds
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 10.0)) {
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);

            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

            // Step 1:  Drive forward for 5.0 seconds -- get in position to launch balls
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 5.0)) {
                leftMotor.setPower(FORWARD_SPEED);
                rightMotor.setPower(FORWARD_SPEED);

                telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            // Step 2:  Run ball launcher for 2 seconds -- launch 1st ball
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                leftMotor.setPower(0.0);
                rightMotor.setPower(0.0);
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

            // Step 4:  Drive forward for 0.5 seconds -- tap cap-ball off center
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                intakeMotor.setPower(0.0);
                shooterMotor.setPower(0.0);
                leftMotor.setPower(FORWARD_SPEED);
                rightMotor.setPower(FORWARD_SPEED);

                telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }

            //  Step 5:  Stop and wait
            leftMotor.setPower(0.0);
            rightMotor.setPower(0.0);
            shooterMotor.setPower(0.0);
            intakeMotor.setPower(0.0);

            telemetry.addData("Path", "Complete");
            telemetry.update();
            sleep(1000);
        }
    }
}
