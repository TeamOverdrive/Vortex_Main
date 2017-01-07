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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
/* import com.qualcomm.robotcore.hardware.DcMotorSimple; */
/* import com.qualcomm.robotcore.hardware.HardwareMap;  */
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode started with the common Pushbot hardware class to define the devices on the robot.  The
 * specific hardware for the Vortex OpMode was added to the program and defined.  The original hardware
 * call was to the HardwarePushbot class.  The class will be deactived to this program once everything
 * is transferred and working properly.  Possible update will be to add all device access
 * to a new hardware class similar to the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * This code has been updated by David Turner and Samuel Turner to suit the needs of Team Overdrive.
 * See internal comments for more info on modifications.
 *
 * Utilizes the Init object to initialize motors and sensors as necessary for the teleop mode.
 * See its documentation for more information.
 *
 * @author David Turner
 * @author Samuel Turner
 * @version 2017.1.3
 */

@TeleOp(name="Teleop Main: Vortex", group="Vortex")

public class TeleopMain_Vortex extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor liftMotor;
    private DcMotor shooterMotor;
    private DcMotor intakeMotor;

    /* Declare Servos */
    private Servo pushButton1;
    private Servo pushButton2;
    private Servo ballRelease;
    private Servo distanceFlag;
    private Servo shooterFlag;
    private Servo lineFlag;

    /* Declare Sensors */
    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private OpticalDistanceSensor opticalSensor;
    private GyroSensor gyroSensor;
    private UltrasonicSensor ultrasonicSensor;

    private static final double MID_SERVO       =  0.5 ;

    private int NUM_TICKS = 10;

    /**
     * The method that runs when the game is started.
     * Receives inputs from the remotes and controls the robot accordingly.
     * Maintains checks on certain specific sensors to determine if predetermined states exist.
     */
    @Override
    public void runOpMode() {
        int counter = 0;
        boolean launch = false;
        double          pushOffset  = 0.0 ;                  // push button Servo mid position
        final double    PUSH_SPEED  = 0.02 ;                 // sets rate to move PUSH servo
        double          releaseOffset = 0.0 ;                // release Servo mid position
        final double    RELEASE_SPEED = 0.02 ;               // sets rate to move RELEASE servo

        //Initialize the motors.
        Init init = new Init();
        init.initTeleop(hardwareMap);

        leftMotor = init.getLeftMotor();
        rightMotor = init.getRightMotor();
        shooterMotor = init.getShooterMotor();
        liftMotor = init.getLiftMotor();
        intakeMotor = init.getIntakeMotor();

        pushButton1 = init.getPushButton1();
        pushButton2 = init.getPushButton2();
        distanceFlag = init.getDistanceFlag();
        shooterFlag = init.getShooterFlag();
        lineFlag = init.getLineFlag();
        ballRelease = init.getBallRelease();

        opticalSensor = init.getOpticalSensor();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            RobotLog.d("LinearOpMode received a CancellationException; shutting down this linear op mode");
            if (opticalSensor.getLightDetected() > 0.2) lineFlag.setPosition(0.0);
            else lineFlag.setPosition(0.5);

            /* ===================================================================================*/
            /* Gamepad 1 - Controls */

            /* Drivetrain Controls */
            float leftThrottle = gamepad1.left_stick_y;
            float rightThrottle = gamepad1.right_stick_y;

            /* Clip the left and right throttle values so that they never exceed +/- 1.  */
            leftThrottle = Range.clip(leftThrottle,-1,1);
            rightThrottle = Range.clip(rightThrottle,-1,1);

            /* Scale the throttle values to make it easier to control the robot more precisely at slower speeds.  */
            leftThrottle = (float) scaleInput(leftThrottle);
            rightThrottle = (float) scaleInput(rightThrottle);

            if (Math.abs(leftThrottle) == 0 && Math.abs(rightThrottle) == 0) {
                if (gamepad1.dpad_up) {
                    rightMotor.setPower(0.2);
                    leftMotor.setPower(0.2);
                }
                else if (gamepad1.dpad_down) {
                    rightMotor.setPower(-0.2);
                    leftMotor.setPower(-0.2);
                }
                else {
                    rightMotor.setPower(0.0);
                    leftMotor.setPower(0.0);
                }
            }
            /* Set power to the drive motors  */
            rightMotor.setPower(rightThrottle);
            leftMotor.setPower(leftThrottle);

            // Indicator button push controls
            // Use gamepad left & right Bumpers to push left or right with the push button device
            if (gamepad1.right_bumper) pushButton1.setPosition(0.0);
            else pushButton1.setPosition(0.5);
            if (gamepad1.left_bumper) pushButton2.setPosition(0.5);
            else pushButton2.setPosition(0.0);

            /* ===================================================================================*/
            /* Gamepad 2 - Controls */

            /* Intake Controls */
            if (gamepad2.right_bumper || (gamepad1.right_trigger > 0))
                intakeMotor.setPower(1.0);
            else if (gamepad2.left_bumper || (gamepad1.left_trigger > 0))
                intakeMotor.setPower(-1.0);
            else
                intakeMotor.setPower(0.0);

            /*  Shooter Control --
            *   "A" gamepad button turns on the shooter and
            *   "B" gamepad button turns the shooter power off.   */
            if (gamepad2.a) {
                shooterMotor.setPower(-1.0);
                launch = true;
            }
            else if (gamepad2.b) {
                shooterMotor.setPower(0.0);
                launch = false;
            }

            // Ball Release Control
            if (gamepad2.x) ballRelease.setPosition(0.0);
            else ballRelease.setPosition(0.2);

            //Scissor Lift Controls for handling the cap ball lifting
            float leftRange = gamepad2.left_stick_y;

            /* Clip the left and right throttle values so that they never exceed +/- 1.  */
            leftRange = Range.clip(leftRange,-1,1);

            /* Scale the throttle values to make it easier to control the robot more precisely at slower speeds.  */
            leftRange = (float) scaleInput(leftRange);

            /* Set power to the drive motors  */
            liftMotor.setPower(leftRange);


            // Send telemetry message to signify robot running;
         /*   telemetry.addData("claw",  "Offset = %.2f", clawOffset);  */
            telemetry.addData("status left_drive", leftMotor);
            telemetry.addData("status right_drive", rightMotor);
            telemetry.addData("status intake", intakeMotor);
            telemetry.addData("shooter", shooterMotor);
            telemetry.addData("lift", liftMotor);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
         /*   robot.waitForTick(40);  */
        }
    }

    /**
     * Sets control scaling for the drive train.
     * @param dVal The value to be scaled.
     * @return The value to which the function has scaled the input.
     */
    double scaleInput(double dVal)   {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16)  {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0)  {
            dScale = -scaleArray[index];
        }  else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
