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
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A super class for all autonomous codes.
 * For each year write methods that are useful for basic tasks (eg. shoot a ball, push a button,
 * etc.) and then use those methods in each autonomous program that way. All defined motors are
 * available to the subclasses through polymorphism. Define all other autonomous classes as follows:
 * public class 'name' extends AutoSuper {
 * super.runOpMode();
 *
 * @author Samuel Turner
 * @verson 2017.1.3
 */

public class AutoSuper extends LinearOpMode {
    protected ElapsedTime runtime = new ElapsedTime();// FORWARD_SPEED was running the robot in reverse to the TeleOp program setup.  Speed is reversed to standardize the robot orientation.
    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 0.75;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;  //modified speed from 0.6
    static final double TURN_SPEED = 0.3;  //modified turn speed from 0.5
    static final double PUSH_MAX1 = 0.0;
    static final double PUSH_MAX2 = 0.6;
    static final double PUSH_MIN1 = 0.6;
    static final double PUSH_MIN2 = 0.0;

    static final double WHITE_THRESHOLD = 0.2;  // spans between 0.1 - 0.5 from dark to light
    static final double APPROACH_SPEED = 0.2;//0.3;  // Approach speed is set to allow the robot to stop on the white line and not go past
    static final double HEADING_THRESHOLD = 1;  // As tight as possible with an integer gyro - increased from 1
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive but less stable - increased from 0.1
    static final double P_DRIVE_COEFF = 0.15;   // Larger is more responsive but less stable
    static double SONIC_RANGE;
    static final double targetDist = 7.0;

    //The following objects are all protected and thus can only be accessed by the autonomous sub-classes.
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
    protected Servo forkRelease;
    protected Servo lineFlag;

    /* Declare Sensors*/
    ModernRoboticsI2cColorSensor color;
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
        forkRelease = init.getForkRelease();
        lineFlag = init.getLineFlag();

        //Define the sensors.
        colorSensor1 = init.getColorSensor1();
        colorSensor2 = init.getColorSensor2();
        opticalSensor = init.getOpticalSensor();
        gyroSensor = init.getGyroSensor();
        ultrasonicSensor = init.getUltrasonicSensor();
    }

    public void turnGyroAbs(int deg) {
        int delta = (deg - gyroSensor.getHeading() + 360) % 360;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (delta >= 180) {
            turnGyroAbsCCW(deg);
        }
        else {
            turnGyroAbsCW(deg);
        }
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turnGyroAbsCCW(int deg) {
        double delta = (deg - gyroSensor.getHeading() + 360) % 360;
        runtime.reset();
        while (opModeIsActive() && delta < 355 && delta > 180 && runtime.seconds() < 4.0) {
            if ((-delta + 360) > 90) {
                leftMotor.setPower(0.5);
                rightMotor.setPower(-0.5);
            }
            else {
                leftMotor.setPower(((-delta + 360) / 180) + 0.3);
                rightMotor.setPower(-(((-delta + 360) / 180) + 0.3));
            }
            delta = (deg - gyroSensor.getHeading() + 360) % 360;
            telemetry.addData("Degrees off", delta);
            telemetry.addData("Speed modifier", (delta/180));
            telemetry.addData("Turning", "counterclockwise");
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    //delta = ((deg + 360) - heading) % 360
    private void turnGyroAbsCW(int deg) {
        double delta = (deg - gyroSensor.getHeading() + 360) % 360;
        runtime.reset();
        while (opModeIsActive() && delta > 5 && delta < 180 && runtime.seconds() < 4.0) {
            if ((delta / 180) > 90) {
                leftMotor.setPower(-0.5);
                rightMotor.setPower(0.5);
            }
            else {
                leftMotor.setPower(-((delta / 180) + 0.3));
                rightMotor.setPower(((delta / 180) + 0.3));
            }
            delta = (deg - gyroSensor.getHeading() + 360) % 360;
            telemetry.addData("Degrees off", delta);
            telemetry.addData("Speed modifier", (delta/180));
            telemetry.addData("Turning", "clockwise");
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    /**
     * Turns relative to the initial heading.
     *
     * @param degrees The degrees relative to the starting heading.
     * @precondition The gyro has been calibrated properly in user code.
     */
    public void turnWithGyro(int degrees) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 4.0) {

        }
    }

    // Drive Easy Encoder Drive
    // set the speed and distance on the left and right motors
    protected void encoderDriveEasy(double speed, double l, double r, double timeout) {
        double endPercent = 6 / Math.abs(l); // Percent of the drive that is the last 4 inches
        encoderDrive(speed, l * (1 - endPercent), r * (1 - endPercent), timeout * (1 - endPercent));
        encoderDrive(DRIVE_SPEED * 0.2, l * endPercent, r * endPercent, timeout * (1 - endPercent));
    }

    /**
     * Methods for Beacon Pushing
     * <p>
     * Pushes the buttons on the beacon starting by moving forward, then reversing to
     * the second beacon.
     *
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconForward(boolean red) {
        driveToWLine(-1);
        turnGyroAbs(0);
        if (!pushButton(red)) pushButton(red);
        encoderDrive(DRIVE_SPEED * 0.8, 26.0, 26.0, 4.0);
        encoderDrive(DRIVE_SPEED * 0.4, 4.0, 4.0, 3.0);
        driveToWLine(1);
        turnGyroAbs(0);
        if (!pushButton(red)) pushButton(red);
    }

    /**
     * Pushes the buttons on the beacon starting by moving backward, then moving forward
     * to the second beacon.
     *
     * @param red A boolean answer to whether or not the desired color is red.
     */
    public void pushBeaconBackward(boolean red) {
        driveToWLineBlue(1);
        turnGyroAbs(0);
        if (!pushButton(red)) pushButton(red);
        encoderDrive(DRIVE_SPEED * 0.8, -34.0, -34.0, 4.0);
        encoderDrive(DRIVE_SPEED * 0.4, -6.0, -6.0, 3.0);
        driveToWLineBlue(-1);
        turnGyroAbs(0);
        if (!pushButton(red)) pushButton(red);
    }

    /**
     * Pushes the button on the beacon based on alliance color and the randomized side that should
     * be used.
     *
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
                sleep(1200);
                pushButton1.setPosition(PUSH_MIN1);
            } else if (colorSensor2.red() >= 2) {
                pushButton2.setPosition(PUSH_MAX2);
                sleep(1200);
                pushButton2.setPosition(PUSH_MIN2);
            }
            sleep(500);
            if (colorSensor1.red() >= 2 && colorSensor2.red() >= 2) {
                return true;
            }
        } else {
            if (colorSensor1.blue() >= 2 && colorSensor2.blue() >= 2) {
                return true;
            }
            if (colorSensor1.blue() >= 2) {
                pushButton1.setPosition(PUSH_MAX1);
                sleep(1000);
                pushButton1.setPosition(PUSH_MIN1);
            } else if (colorSensor2.blue() >= 2) {
                pushButton2.setPosition(PUSH_MAX2);
                sleep(1000);
                pushButton2.setPosition(PUSH_MIN2);
            }
            sleep(500);
            if (colorSensor1.blue() >= 2 && colorSensor2.blue() >= 2) {
                return true;
            }
        }
        return false;
    }

    public boolean driveToWLine(int dir) {
        //final double MAX_CHANGE = 2.0; // Max expected cm change per cycle. No idea what to expect here. Tune up or down. This should be somewhere a bit less than how far robot can drive in 1/10 second.
        double prevDist;
        double curDist;
        int desiredAngle;
        int curAngle;
        double deltaFromTarget;

        prevDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
        sleep(100);
        leftMotor.setPower((APPROACH_SPEED) * dir);
        rightMotor.setPower((APPROACH_SPEED) * dir);
        runtime.reset();

        while (opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 8) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            String out = Double.toString(opticalSensor.getLightDetected());
            RobotLog.d(out);

            // First calculate how fast we want to be moving towards target
            curDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
            curDist *= Math.cos(Math.toRadians(gyroSensor.getHeading()));
            deltaFromTarget = curDist - targetDist; // positive if currently greater than target

            if (deltaFromTarget > 13.0) { // Far from target
                desiredAngle = dir *(0 + 15);
            } else if (deltaFromTarget > 2.0) { // Getting close to target
                desiredAngle = dir * (0 + (int) (((deltaFromTarget - 2.0)/ (10 - 2.0)) * 15)); // between +10 and -10 degrees
            } else if (deltaFromTarget < -2.0) { // Went past target **Consider changing this value
                desiredAngle = dir * (0 + (int) (((deltaFromTarget - 2.0) / (10 - 2.0)) * 10));
            } else { // On target
                desiredAngle = 0;
            }

            //if (desiredAngle < 0) desiredAngle += 360;

            curAngle = gyroSensor.getHeading();

            if (curAngle > 180) curAngle -= 360;

            if (desiredAngle < curAngle - 1) {
                leftMotor.setPower((APPROACH_SPEED + (APPROACH_SPEED * 0.5 * dir)) * dir);
                rightMotor.setPower((APPROACH_SPEED - (APPROACH_SPEED * 0.5 * dir)) * dir);
            } else if (desiredAngle > curAngle + 1) {
                leftMotor.setPower((APPROACH_SPEED - (APPROACH_SPEED * 0.5 * dir)) * dir);
                rightMotor.setPower((APPROACH_SPEED + (APPROACH_SPEED * 0.5 * dir)) * dir);
            } else {
                leftMotor.setPower((APPROACH_SPEED) * dir);
                rightMotor.setPower((APPROACH_SPEED) * dir);
            }
            telemetry.addData("currrent angle", curAngle);
            telemetry.addData("desired angle", desiredAngle);
            telemetry.addData("current distance", curDist);
            telemetry.addData("delta from target", deltaFromTarget);
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        //Turn to zero at this point
        return runtime.seconds() < 3;
    }

    public boolean driveToWLineBlue(int dir) {
        //final double MAX_CHANGE = 2.5; // Max expected cm change per cycle. No idea what to expect here. Tune up or down. This should be somewhere a bit less than how far robot can drive in 1/10 second.
        double prevDist;
        double curDist;
        int desiredAngle;
        int curAngle;
        double deltaFromTarget;

        prevDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
        sleep(100);
        leftMotor.setPower((APPROACH_SPEED) * dir);
        rightMotor.setPower((APPROACH_SPEED) * dir);
        runtime.reset();

        while (opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 8) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            String out = Double.toString(opticalSensor.getLightDetected());
            RobotLog.d(out);

            // First calculate how fast we want to be moving towards target
            curDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
            curDist *= Math.cos(Math.toRadians(gyroSensor.getHeading()));
            deltaFromTarget = curDist - targetDist; // positive if currently greater than target

            if (deltaFromTarget > 13.0) { // Far from target
                desiredAngle = dir *(180 + 15);
            } else if (deltaFromTarget > 2.5) { // Getting close to target
                desiredAngle = dir * (180 + (int) (((deltaFromTarget - 2.5)/ (10 - 2.5)) * 15)); // between +10 and -10 degrees
            } else if (deltaFromTarget < -2.5) { // Went past target **Consider changing this value
                desiredAngle = dir * (180 + (int) (((deltaFromTarget - 2.5) / (10 - 2.5)) * 10));
            } else { // On target
                desiredAngle = 180;
            }

            //if (desiredAngle < 0) desiredAngle += 360;

            curAngle = gyroSensor.getHeading();

            if (curAngle > 180) curAngle -= 360;

            if (desiredAngle < curAngle - 1) {
                leftMotor.setPower((APPROACH_SPEED + (APPROACH_SPEED * 0.5 * dir)) * dir);
                rightMotor.setPower((APPROACH_SPEED - (APPROACH_SPEED * 0.5 * dir)) * dir);
            } else if (desiredAngle > curAngle + 1) {
                leftMotor.setPower((APPROACH_SPEED - (APPROACH_SPEED * 0.5 * dir)) * dir);
                rightMotor.setPower((APPROACH_SPEED + (APPROACH_SPEED * 0.5 * dir)) * dir);
            } else {
                leftMotor.setPower((APPROACH_SPEED) * dir);
                rightMotor.setPower((APPROACH_SPEED) * dir);
            }
            telemetry.addData("currrent angle", curAngle);
            telemetry.addData("desired angle", desiredAngle);
            telemetry.addData("current distance", curDist);
            telemetry.addData("delta from target", deltaFromTarget);
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        //Turn to zero at this point
        return runtime.seconds() < 3;
    }


    /**
     * Method for launching balls
     * Launch the balls loaded in the hopper at the center structure.
     * Takes approximately 2 seconds per ball.
     *
     * @param num The number of balls in the hopper. This is a positive integer <= 2
     */
    public void launchBalls(int num) {
        intakeMotor.setPower(1.0);
        for (int i = 0; i < num; i++) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < 0.9)) {
                shooterMotor.setPower(-1.0);
            }
            // shooterMotor.setPower(0.0);
            if (opModeIsActive()) {
                ballRelease.setPosition(0.0); //Set to open the gate to release the second ball
                sleep(500);  //Set to hold open the gate to allow the second ball to pass the gate
            }
        }
        shooterMotor.setPower(0.0);
        intakeMotor.setPower(0.0);
        ballRelease.setPosition(0.4);
    }

    /**
     * Method gyro calibration
     * <p>
     * Autonomous initialization routine that calibrates the gyro and resets
     * the drive motor encoders before the start of the autonomous program.
     * <p>
     * This code is copied and then modified from the sample code.
     */
    public void calibrateGyro() {
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyroSensor.calibrate();                                     // Calibrate Gyro Sensor
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset left motor encoder
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset right motor encoder
        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyroSensor.isCalibrating()) {
            sleep(50);
            idle();
        }
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Method to perform a relative move, based on encoder counts.
     * Encoders are not reset as the move is based on the current position.
     * Move will stop if any of three conditions occur:
     * 1) Move gets to the desired position
     * 2) Move runs out of time
     * 3) Driver stops the opmode running.
     *
     * @param speed       The speed that the motors are moving.
     * @param leftInches  The distance that the robot should move to the left.
     * @param rightInches The distance that the robot should move to the right.
     * @param timeoutS    The amount of time this method is allowed to execute.
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
            int counter1 = 0;
            int counter2 = 0;

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
                counter1++;
                if (Math.abs(newLeftTarget - leftMotor.getCurrentPosition()) < (6.0 * COUNTS_PER_INCH)) {
                    leftMotor.setPower(Math.abs(speed * 0.3));
                    rightMotor.setPower(Math.abs(speed * 0.3));
                    counter2++;
                }
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Distance", Math.abs(newLeftTarget - leftMotor.getCurrentPosition()));
                telemetry.addData("While counter", counter1);
                telemetry.addData("If counter", counter2);
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

    //Theoretically unusable methods, DO NOT USE THESE.
    /**
     * Methods for Gyro Sensor turning excluding turn correction / adjustments
     * <p>
     * Turns the robot the given number of degrees in the given direction.
     * Negative is left and positive is right.
     * Don't use degrees > 180 for safety reasons.
     *
     * @param deg The number of degrees to turn.
     */
    public void turnGyroRelL(int deg) {
        sleep(250);
        int tDeg = ((gyroSensor.getHeading() + deg)) % 360;
        while (gyroSensor.getHeading() != tDeg) {
            leftMotor.setPower(-DRIVE_SPEED * 0.2);
            rightMotor.setPower(DRIVE_SPEED * 0.2);
        }
        sleep(100);
    }

    /**
     * Methods for Encoder based turns -- these methods do not use the Gyro Sensors to
     * make corrections for over/under turning.
     */
    //Encoder 90 degree left turn
    public void turn90L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, 13.0, -13.0, 3.0); //reduced from 13.0 for change in gearing
        sleep(100);
    }

    //Encoder 90 degree right turn
    public void turn90R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, -13.0, 13.0, 3.0); //reduced from 13.0 for change in gearing
        sleep(100);
    }

    //Red Side 90 degree left turn
    public void turn90L_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.3, 11.1, -11.1, 3.0); //reduced from 11.5 for change in gearing
        sleep(100);
    }

    //Red Side 90 degree right turn
    public void turn90R_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.4, -11.1, 11.1, 3.0); //reduced from 11.5 for change in gearing
        sleep(100);
    }


    //Encoder 45 degree left turn
    public void turn45L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.5, 6.5, -6.5, 3.0); //reduced from 6.5 for change in gearing
        sleep(100);
    }

    //Encoder 45 degree right turn
    public void turn45R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.5, -6.5, 6.5, 3.0); //reduced from 6.5 for change in gearing
        sleep(100);
    }

    //Encoder 30 degree left turn
    public void turn30L()  {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.8, 4.5, -4.5, 2.0);
        sleep(100);
    }

    //Encoder 30 degree right turn
    public void turn30R()  {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.8, -4.5, 4.5, 2.0);
        sleep(100);
    }

    //Encoder 35 degree left turn
    public void turn35L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.5, 5.0, -5.0, 3.0); //reduced from 6.5 for change in gearing
        sleep(100);
    }

    //Encoder 35 degree right turn
    public void turn35R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.5, -5.0, 5.0, 3.0); //reduced from 6.5 for change in gearing
        sleep(100);
    }


    //Red Side 45 degree left turn
    public void turn45L_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.4, 6.5, -6.5, 3.0); //reduced from 10.5 for change in gearing
        sleep(100);
    }

    //Red Side 45 degree right turn
    public void turn45R_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.4, -6.5, 6.5, 3.0); //reduced from 10.5 for change in gearing
        sleep(100);
    }

    //Encoder 135 degree left turn
    public void turn135L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, 19.5, -19.5, 3.0); //reduced from 19.5 for change in gearing
        sleep(100);
    }

    //Encoder 135 degree right turn
    public void turn135R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, -19.5, 19.5, 3.0); //reduced from 19.5 for change in gearing
        sleep(100);
    }


    //Red Side 135 degree left turn
    public void turn135L_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.4, 18.4, -18.4, 3.0); //reduced from 17.5 for change in gearing
        sleep(100);
    }

    //Red Side 135 degree right turn
    public void turn135R_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED * 0.3, -20.8, 20.8, 3.0); //reduced from 17.5 for change in gearing
        sleep(100);
    }

    //Encoder 180 degree left turn
    public void turn180L() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, 25.0, -25.0, 3.0); //reduced from 22.0 for change in gearing
        sleep(100);
    }

    //Encoder 180 degree right turn
    public void turn180R() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, -26.0, 26.0, 3.0); //reduced from 22.0 for change in gearing
        sleep(100);
    }


    //Red Side 180 degree left turn
    public void turn180L_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, 26.0, -26.0, 3.0); //reduced from 22.0 for change in gearing
        sleep(100);
    }

    //Red Side 180 degree right turn
    public void turn180R_RED() {
        sleep(250);
        encoderDrive(DRIVE_SPEED / 2, -26.0, 26.0, 3.0); //reduced from 22.0 for change in gearing
        sleep(100);
    }

    public void turnGyroAbsL(int deg) {
        telemetry.addData("Gyro_Heading", gyroSensor.getHeading());
        telemetry.update();
        while (opModeIsActive() && gyroSensor.getHeading() != deg) {
            leftMotor.setPower(DRIVE_SPEED * 0.2);
            rightMotor.setPower(-DRIVE_SPEED * 0.2);
            telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    public void turnGyroAbsR(int deg) {
        while (opModeIsActive() && gyroSensor.getHeading() != deg) {
            leftMotor.setPower(-DRIVE_SPEED * 0.2);
            rightMotor.setPower(DRIVE_SPEED * 0.2);
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }


    /* Method approachWall is used for course adjustments to remain at the correct distance
     *  from the wall during autonomous driving.  The method uses the ultrasonic sensor to
     *  determine the distance from the wall and to make a course correction by adjusting
     *  motor speed.
     */

    public void approachWall() throws InterruptedException {
        while (opModeIsActive()) {
            SONIC_RANGE = ultrasonicSensor.getDistance(DistanceUnit.CM);
            leftMotor.setPower(0.5 - ((8.0 - SONIC_RANGE) * 0.05));
            rightMotor.setPower(0.5 + ((8.0 - SONIC_RANGE) * 0.05));
        }
        driveToWLine(1);
    }

    /**
     * Method gyroDrive
     * <p>
     * Gyro Sensor Driving Setting and Methods to allow for gyro driving in autonomous.
     * This code is copied and then modified from the sample code.
     * <p>
     * Drive with the gyro on a fixed compass bearing based on encoder counts.  The target
     * speed for forward motion should allow for fluctuations to allow for heading adjustments.
     * The distance is set in inches from the current position.  Negative distance moves backwards.
     * The angle is in degrees relative to the last gyro reset.  0 is forward, +ve is counter
     * clockwise from forward, -ve is clockwise from forward position.
     * <p>
     * This code is copied and then modified from the sample code.
     */

    public void gyroDrive(double speed, double distance, double angle) {
        int newLeftTarget;
        int newRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newLeftTarget = leftMotor.getCurrentPosition() + moveCounts;
            newRightTarget = rightMotor.getCurrentPosition() + moveCounts;

            //Set Target and turn on Run_To_Position
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start moving
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            while (opModeIsActive() && (leftMotor.isBusy() && rightMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed + steer;
                rightSpeed = speed - steer;

                // Normalize speeds if any one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftMotor.setPower(leftSpeed);
                rightMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Actual", "%7d:%7d", leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method gyroTurn
     * <p>
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     *              <p>
     *              This code is copied and then modified from the sample code.
     */

    public void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method gyroHold
     * <p>
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     *                 <p>
     *                 This code is copied and then modified from the sample code.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }


    /**
     * Gyro turn correction method used as a sub-routine in gyroTurn method
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return This code is copied and then modified from the sample code.
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftMotor.setPower(leftSpeed);
        rightMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }


    /**
     * Sub-routine for angle error identification used in other methods
     * <p>
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     * <p>
     * This code is copied and then modified from the sample code.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyroSensor.getIntegratedZValue();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * Sub-routine for steering adjustments used in other methods
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    //This is a legacy driveToWLine version.
    //Uncomment at your own risk (preferably delete all of this).
    /**
     * Method for white line identification and movement between lines
     * <p>
     * Drives until it reaches a white line on the ground.
     * ONLY USE 1 OR -1 AS INPUT VALUES.
     *
     * @param dir Positive for forward, negative for reverse.
     *//*
    public boolean driveToWLine(int dir) {
        final double MAX_CHANGE = 2.5; // Max expected cm change per cycle. No idea what to expect here. Tune up or down. This should be somewhere a bit less than how far robot can drive in 1/10 second.
        double prevDist;
        double curDist;
        double curChange;
        double targetChange = 0.0;
        double desiredChange;
        double targetDist = 10.0;
        double deltaFromTarget;

        prevDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
        sleep(100);
        leftMotor.setPower((APPROACH_SPEED) * dir);
        rightMotor.setPower((APPROACH_SPEED) * dir);
        runtime.reset();
            while(opModeIsActive() && (opticalSensor.getLightDetected() < 0.08) && runtime.seconds() < 8 ) {
            telemetry.addData("Light Level", opticalSensor.getLightDetected());
            String out = Double.toString(opticalSensor.getLightDetected());
            RobotLog.d(out);
                // First calculate how fast we want to be moving towards target
                curDist = ultrasonicSensor.getDistance(DistanceUnit.CM);
                deltaFromTarget = curDist - targetDist; // positive if currently greater than target
                if (deltaFromTarget > 10.0) { // Far from target
                    desiredChange = -1 * (MAX_CHANGE / 100);
                    leftMotor.setPower((APPROACH_SPEED + desiredChange) * dir);
                    rightMotor.setPower((APPROACH_SPEED - desiredChange) * dir);
                } else if (deltaFromTarget > 1.5) { // Getting close to target
                    desiredChange = -1 * (deltaFromTarget / 100.0) * MAX_CHANGE; // between zero and negative max based on how far from target
                    leftMotor.setPower((APPROACH_SPEED + desiredChange) * dir);
                    rightMotor.setPower((APPROACH_SPEED - desiredChange) * dir);
                } else if (deltaFromTarget < -1.5) { // Went past target
                    desiredChange = -1 * (deltaFromTarget / 100.0) * MAX_CHANGE; // between zero and positive max based on how far from target
                    leftMotor.setPower((APPROACH_SPEED + desiredChange) * dir);
                    rightMotor.setPower((APPROACH_SPEED - desiredChange) * dir);
                } else { // On target
                    desiredChange = 0;
                    leftMotor.setPower((APPROACH_SPEED + desiredChange) * dir);
                    rightMotor.setPower((APPROACH_SPEED - desiredChange) * dir);
                }
               /* if (ultrasonicSensor.getDistance(DistanceUnit.CM) >= 17) {
                    leftMotor.setPower((0.9) * dir);
                    rightMotor.setPower((0.1) * dir);
                } else if (ultrasonicSensor.getDistance(DistanceUnit.CM) >= 9) {
                    leftMotor.setPower((0.5 + ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05)) * dir);
                    rightMotor.setPower((0.5 - ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05)) * dir);
                } else if (ultrasonicSensor.getDistance(DistanceUnit.CM) <= 7) {
                    leftMotor.setPower((0.5 + ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05)) * dir);
                    rightMotor.setPower((0.5 - ((8.0 - ultrasonicSensor.getDistance(DistanceUnit.CM)) * 0.05)) * dir);
                } else {
                    leftMotor.setPower((DRIVE_SPEED/2) * dir);
                    rightMotor.setPower((DRIVE_SPEED/2) * dir);
                }
                telemetry.update();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
        return runtime.seconds() < 3;
    }
    */
}
