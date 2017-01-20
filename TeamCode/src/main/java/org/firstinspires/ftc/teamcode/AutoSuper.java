package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A super class for all autonomous codes.
 * For each year write methods that are useful for basic tasks (eg. shoot a ball, push a button,
 * etc.) and then use those methods in each autonomous program that way. All defined motors are
 * available to the subclasses through polymorphism. Define all other autonomous classes as follows:
 *  public class 'name' extends AutoSuper {
 *      super.runOpMode();
 * @author Samuel Turner
 * @verson 2017.1.3
 */

public class AutoSuper extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();// FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = -0.5;
    static final double PUSH_MAX1 = 0.0;
    static final double PUSH_MAX2 = 0.5;
    static final double PUSH_MIN1 = 0.5;
    static final double PUSH_MIN2 = 0.0;

    static final double     WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double     APPROACH_SPEED  = 0.5;

    //The following objects are all protected an thus can only be accessed by the autonomous sub-classes.
    /* Declare OpMode members. */
    protected DcMotor leftMotor;
    protected DcMotor rightMotor;
    protected DcMotor liftMotor;
    protected DcMotor shooterMotor;
    protected DcMotor intakeMotor;

    /* Declare Servos */
    protected Servo pushButton1;
    protected Servo pushButton2;
    protected Servo ballRelease;
    protected Servo distanceFlag;
    protected Servo shooterFlag;
    protected Servo lineFlag;
    ModernRoboticsI2cColorSensor color;
    /* Declare Sensors*/
    protected ColorSensor colorSensor1;
    protected ColorSensor colorSensor2;
    protected OpticalDistanceSensor opticalSensor;
    protected ModernRoboticsI2cGyro gyroSensor;
    protected ModernRoboticsI2cRangeSensor ultrasonicSensor;

    /**
     * Defines the standard starting setup for the Autonomous classes.
     */
    @Override
    public void runOpMode() {
        Init init = new Init();
        init.initAuto(hardwareMap, telemetry, this);

        //Define the motors.
        leftMotor = init.getLeftMotor();
        rightMotor = init.getRightMotor();
        liftMotor = init.getLiftMotor();
        shooterMotor = init.getShooterMotor();
        intakeMotor = init.getIntakeMotor();

        //Define the servos.
        pushButton1 = init.getPushButton1();
        pushButton2 = init.getPushButton2();
        ballRelease = init.getBallRelease();
        distanceFlag = init.getDistanceFlag();
        shooterFlag = init.getShooterFlag();
        lineFlag = init.getLineFlag();

        //Define the sensors.
        colorSensor1 = init.getColorSensor1();
        colorSensor2 = init.getColorSensor2();
        opticalSensor = init.getOpticalSensor();
        gyroSensor = init.getGyroSensor();
        ultrasonicSensor = init.getUltrasonicSensor();
    }

    public void turn90L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, 12.0, -12.0, 3.0);
        sleep(100);
    }

    public void turn90R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, -12.0, 12.0, 3.0);
        sleep(100);
    }

    public void turn45L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, 6.0, -6.0, 3.0);
        sleep(100);
    }

    public void turn45R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, -6.0, 6.0, 3.0);
        sleep(100);
    }

    public void turn135L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, 18.0, -18.0, 3.0);
        sleep(100);
    }

    public void turn135R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED/2, -18.0, 18.0, 3.0);
        sleep(100);
    }


    /**
     * Turns the robot the given number of degrees in the given direction.
     * Negative is left and positive is right.
     * Don't use degrees > 180 for safety reasons.
     * @param deg The number of degrees to turn.
     */
    public void turnGyroRelL(int deg) {
        sleep(250);
        int tDeg = ((gyroSensor.getHeading() + deg)) % 360;
        while (gyroSensor.getHeading() != tDeg) {
            leftMotor.setPower(-DRIVE_SPEED * 0.4);
            rightMotor.setPower(DRIVE_SPEED * 0.4);
        }
        sleep(100);
    }

    public void turnGyroAbsL(int deg) {
        while(opModeIsActive() && gyroSensor.getHeading() != deg) {
            leftMotor.setPower(-DRIVE_SPEED * 0.3);
            rightMotor.setPower(DRIVE_SPEED * 0.3);
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void turnGyroAbsR(int deg) {
        while(opModeIsActive() && gyroSensor.getHeading() != deg) {
            leftMotor.setPower(DRIVE_SPEED * 0.3);
            rightMotor.setPower(-DRIVE_SPEED * 0.3);
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    /**
     * Pushes the buttons on the beacon starting by moving forward, then reversing to
     * the second beacon.
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconForward(boolean red) {
        driveToWLine(1);
        if (!pushButton(red)) pushButton(red);
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0);
        driveToWLine(-1);
        if (!pushButton(red)) pushButton(red);
    }

    /**
     * Pushes the buttons on the beacon starting by moving backward, then moving forward
     * to the second beacon.
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconBackward(boolean red) {
        driveToWLine(-1);
        if (!pushButton(red)) pushButton(red);
        encoderDrive(DRIVE_SPEED, -24.0, -24.0, 5.0);
        driveToWLine(1);
        if (!pushButton(red)) pushButton(red);
    }

    /**
     * Pushes the button on the beacon based on alliance color and the randomized side that should
     * be used.
     * @param red A boolean representing whether or not the desired color is red.
     */
    public boolean pushButton(boolean red) {
        colorSensor1.enableLed(false);
        colorSensor2.enableLed(false);
        telemetry.addData("color1 red", colorSensor1.red());
        telemetry.addData("color1 blue", colorSensor1.blue());
        telemetry.addData("color2 red", colorSensor2.red());
        telemetry.addData("color2 blue", colorSensor2.blue());
        telemetry.update();
        if (red) {
            if (colorSensor1.red() >= 2 && colorSensor2.red() >= 2) {
                return true;
            }
            if (colorSensor1.red() >= 2) {
                pushButton1.setPosition(PUSH_MAX1);
                sleep(750);
                pushButton1.setPosition(PUSH_MIN1);
            }
            else if (colorSensor2.red() >= 2){
                pushButton2.setPosition(PUSH_MAX2);
                sleep(750);
                pushButton2.setPosition(PUSH_MIN2);
            }
            sleep(500);
            if (colorSensor1.red() >= 2 && colorSensor2.red() >= 2) {
                return true;
            }
        }
        else {
            if (colorSensor1.blue() >= 2 && colorSensor2.blue() >= 2) {
                return true;
            }
            if (colorSensor1.blue() >= 2) {
                pushButton1.setPosition(PUSH_MAX1);
                sleep(750);
                pushButton1.setPosition(PUSH_MIN1);
            }
            else if (colorSensor2.blue() >= 2){
                pushButton2.setPosition(PUSH_MAX2);
                sleep(750);
                pushButton2.setPosition(PUSH_MIN2);
            }
            sleep(500);
            if (colorSensor1.blue() >= 2 && colorSensor2.blue() >= 2) {
                return true;
            }
        }
        return false;
    }

    /**
     * Drives until it reaches a white line on the ground.
     * ONLY USE 1 OR -1 AS INPUT VALUES.
     * @param dir Positive for forward, negative for reverse.
     */
    public boolean driveToWLine(int dir) {
        leftMotor.setPower(-(DRIVE_SPEED / 2) * dir);
        rightMotor.setPower(-(DRIVE_SPEED / 2) * dir);
        runtime.reset();
        while(opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 5) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            String out = Double.toString(opticalSensor.getLightDetected());
            RobotLog.d(out);
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        return runtime.seconds() < 3;
    }

    /**
     * Launch the balls loaded in the hopper at the center structure.
     * Takes approximately 4 seconds per ball.
     * @param num The number of balls in the hopper. This is a positive integer <= 2
     */
    public void launchBalls(int num) {
        intakeMotor.setPower(0.0);
        for (int i = 0; i < num; i++) {

            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                shooterMotor.setPower(-1.0);
            }
            shooterMotor.setPower(0.0);
            if (opModeIsActive()) {
                sleep(1000);  //Set the gate open process to delay for 1st ball to be launched
                ballRelease.setPosition(0.0); //Set to open the gate to release the second ball
                sleep(500);  //Set to hold open the gate to allow the second ball to pass the gate
                ballRelease.setPosition(0.4);  //Set to close the gate after the second ball is released
            }
        }
        intakeMotor.setPower(0.0);
    }

    /**
     * Calibrates the gyro.
     * This code is copied and pasted from the sample code.
     */
    public void calibrateGyro() {
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyroSensor.calibrate();
        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyroSensor.isCalibrating())  {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();
    }

    /**
     * Turns relative to the initial heading.
     * @precondition The gyro has been calibrated properly in user code.
     * @param degrees The degrees relative to the starting heading.
     */
    public void turnWithGyro(int degrees) {
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < 4.0) {

        }
    }

    public void approachWall() {
        while(opModeIsActive() && ultrasonicSensor.getDistance(DistanceUnit.CM) >= 8) {
            leftMotor.setPower(0.3);
            rightMotor.setPower(0.5);
            sleep(500);
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.3);
            sleep(500);
        }
        driveToWLine(1);
    }

    /**
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *  @param speed The speed that the motors are moving.
     *  @param leftInches The distance that the robot should move to the left.
     *  @param rightInches The distance that the robot should move to the right.
     *  @param timeoutS The amount of time this method is allowed to execute.
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
