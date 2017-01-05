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

@Autonomous(name="AutoDrive: Enc Inside Do All No Delay", group="Vortex")

public class AutoDrive_Encode_ALL_INSIDE_ND extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;

    private ElapsedTime runtime = new ElapsedTime();

    // FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = -0.6;
    static final double TURN_SPEED = -0.4;

    ColorSensor             colorSensor;      // Modern Robotics Color Sensor
    // LightSensor             lightSensor;      // Primary LEGO Light sensor,
    // OpticalDistanceSensor   lightSensor;   // Alternative MR ODS sensor

    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;


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
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // ********************************************************************************************************

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

// Step 1:  Run with encoders to a position opposite the goal and pause
        encoderDrive(DRIVE_SPEED, -25.5, -25.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
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

        // Step 5:  Position between beacons
        encoderDrive(TURN_SPEED, 27.0, -27.0, 4.0);  // S2: Turn Right 135 degrees with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 60, 60, 5.0);  // S1: Reverse 74 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED, 9.0, -9.0, 4.0);  // S2: Turn Right 45 degrees with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 24.5, 24.5, 4.0);  // S3: Forward 10 Inches with 4 Sec timeout
        encoderDrive(TURN_SPEED, -11.0, 11.0, 4.0);  // S2: Turn Right 90 degrees with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, 12.0, 12.0, 4.0);  // Drive forward along the wall 12 Sec timeout
        sleep(1000);     // pause for servos to move

    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotor.setPower(Math.abs(speed));
            rightMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
