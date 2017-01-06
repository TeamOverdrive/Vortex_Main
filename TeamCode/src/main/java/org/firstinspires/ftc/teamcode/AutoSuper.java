package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

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
    static final double PUSH_MAX1 = 0.5;
    static final double PUSH_MAX2 = 0.0;
    static final double PUSH_MIN1 = 0.0;
    static final double PUSH_MIN2 = 0.5;

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

    /* Declare Sensors*/
    protected ColorSensor colorSensor1;
    protected ColorSensor colorSensor2;
    protected OpticalDistanceSensor opticalSensor;
    protected GyroSensor gyroSensor;
    protected UltrasonicSensor ultrasonicSensor;

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
     //   ballRelease = init.getBallRelease();
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
        sleep(500);
        encoderDrive(DRIVE_SPEED, 12.0, -12.0, 3.0);
        sleep(500);
    }

    public void turn90R() {
        sleep(500);
        encoderDrive(DRIVE_SPEED, -12.0, 12.0, 3.0);
        sleep(500);
    }

    /**
     * Pushes the buttons on the beacon starting by moving forward, then reversing to
     * the second beacon.
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconForward(boolean red) {
        driveToWLine(1);
        for (int i = 0; i < 3; i++) {
            if (pushBeacon(red)) break;
        }
        driveToWLine(-1);
        for (int i = 0; i < 3; i++) {
            if (pushBeacon(red)) break;
        }
    }

    /**
     * Pushes the buttons on the beacon starting by moving backward, then moving forward
     * to the second beacon.
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconBackward(boolean red) {
        driveToWLine(-1);
        for (int i = 0; i < 3; i++) {
            if (pushBeacon(red)) break;
        }
        driveToWLine(1);
        for (int i = 0; i < 3; i++) {
            if (pushBeacon(red)) break;
        }
    }

    /**
     * Activates the beacon based on the team's color.
     * @param red A boolean answer to whether or not the desired color is red.
     * @return Determine whether or not the beacon was successfully pushed.
     */
    private boolean pushBeacon(boolean red) {
        if (red) {
            if (colorSensor1.red() >= 155 && colorSensor2.red() >= 155) {
                return true;
            }
            else if (colorSensor1.blue() >= 155 && colorSensor2.blue() >= 155) {
                sleep(5*1000);
                pushButton(red);
            }
        }
        else {
            if (colorSensor1.blue() >= 155 && colorSensor2.blue() >= 155) {
                return true;
            }
            else if (colorSensor1.blue() >= 155 && colorSensor2.blue() >= 155) {
                sleep(5*1000);
                pushButton(red);
            }
        }
        return false;
    }

    /**
     * Pushes the button on the beacon based on alliance color and the randomized side that should
     * be used.
     * @param red A boolean representing whether or not the desired color is red.
     */
    private void pushButton(boolean red) {
        if (red) {
            if (colorSensor1.red() >= 155) {
                pushButton1.setPosition(PUSH_MAX1); //Fix this value.
                pushButton1.setPosition(PUSH_MIN1);
            }
            else {
                pushButton2.setPosition(PUSH_MAX2);
                pushButton2.setPosition(PUSH_MIN2);
            }
        }
        else {
            if (colorSensor1.blue() >= 155) {
                pushButton1.setPosition(PUSH_MAX1);
                pushButton2.setPosition(PUSH_MIN1);
            }
            else {
                pushButton2.setPosition(PUSH_MAX2);
                pushButton2.setPosition(PUSH_MIN2);
            }
        }
    }

    /**
     * Drives until it reaches a white line on the ground.
     * ONLY USE 1 OR -1 AS INPUT VALUES.
     * @param dir Positive for forward, negative for reverse.
     */
    public boolean driveToWLine(int dir) {
        encoderDrive(DRIVE_SPEED, -8*dir, -8*dir, 2.0);
        leftMotor.setPower(-(DRIVE_SPEED/2) * dir);
        rightMotor.setPower(-(DRIVE_SPEED/2) * dir);
        runtime.reset();
        while(opModeIsActive() && (opticalSensor.getLightDetected() < 0.2) && runtime.seconds() < 3) {
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
        for (int i = 0; i < num; i++) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                shooterMotor.setPower(-1.0);
                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                telemetry.update();
            }
            shooterMotor.setPower(0.0);
            //Allow next ball through
            intakeMotor.setPower(1.0);
            sleep(1000);
            intakeMotor.setPower(0.0);
        }
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
