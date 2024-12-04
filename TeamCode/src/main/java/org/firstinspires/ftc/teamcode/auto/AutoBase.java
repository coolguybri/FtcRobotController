package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 */
public abstract class AutoBase extends LinearOpMode {

    // MAGIC NUMBERS for the motor encoders
    // https://asset.pitsco.com/sharedimages/resources/torquenado_dcmotorspecifications.pdf
    static final float COUNTS_PER_MOTOR_TORKNADO_60 = 1440;  // 24 cycles per revolution, times 60:1 geared down.
    static final float COUNTS_PER_MOTOR_TORKNADO_20 = 480;  // 24 cycles per revolution, times 20:1 geared down.

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    private static final float NUDGE_ANGLE = 4.0f;
    private static final float MOTOR_TURN_SPEED = 0.25f;
    private static final float MOTOR_MOVE_SPEED = 0.6f;
    private static final float WHEEL_DIAMETER = 4.0f;
    private static final long WAIT_TIME = TimeUnit.SECONDS.toMillis(5L);
    private static final float RAT_FUDGE = 0.98f;

    // Instance Members: Motors
    protected boolean doMotors = true;
    private boolean useGear20Motors = true;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private String motorTurnType = "none";
    private float motorTurnDestination = 0.0f;
    private float motorTurnAngleToGo = 0.0f;
    private float motorTurnAngleAdjustedToGo = 0.0f;
    private boolean isDriving = false;
    private float driveAngleOffset = 0.0f;
    private float driveAngleCorrection = 0.0f;
    private int driveBackLeftStart = 0;
    private int driveBackRightStart = 0;
    private int driveFrontLeftStart = 0;
    private int driveFrontRightStart = 0;
    private int driveBackLeftTarget = 0;
    private int driveBackRightTarget = 0;
    private int driveFrontLeftTarget = 0;
    private int driveFrontRightTarget = 0;
    private double driveRightSpeed = 0.0f;
    private double driveLeftSpeed = 0.0f;
    private PIDController motorPid = new PIDController(.05, 0, 0);

    // Instance Members: Gyro
    protected boolean doGyro = true;
    private IMU imu;

    protected boolean doArm = true;
    protected DcMotor arm;
    private int armStart = 0;
    private int armTarget = 0;

    protected boolean doExtend = true;
    protected DcMotor extend;
    private int extendStart = 0;
    private int extendTarget = 0;

    protected boolean doFinger = true;
    protected Servo finger;
    protected Servo wrist;

    protected boolean doThePurplePixelPlopper = true;
    protected Servo thePurplePixelPlopper;

    protected boolean doGate = false;
    protected Servo gate;

    // Instance Members: Core
    private long startTime = System.nanoTime();
    private boolean madeTheRun = false;

    // Instance Members: ObjectDetection
    protected boolean doObjectDetection = false;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    protected List<Recognition> recognitionsList = new ArrayList<>();

    @Override
    public void runOpMode(){
        ratCrewInit();
        waitForStart();

        printStatus();

        ratCrewStart();
        printStatus();
    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    public void ratCrewStart() {
        printStatus();
        ratCrewGo();
        madeTheRun = true;
        printStatus();
    }

    //all Opmodes must override
    public abstract void ratCrewGo();

    // Called once, right after hitting the Init button.
    protected void ratCrewInit() {

        // Init motor state machine.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
        isDriving = false;
        driveBackLeftStart = 0;
        driveBackRightStart = 0;
        driveBackLeftTarget = 0;
        driveBackRightTarget = 0;
        driveRightSpeed = 0.0f;
        driveLeftSpeed = 0.0f;
        driveAngleOffset = 0.0f;
        driveAngleCorrection = 0.0f;

        // TODO: push into initMotor function
        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            backLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftRear");
            backRightDrive = hardwareMap.get(DcMotor.class, "motorRightRear");
            frontLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftFront");
            frontRightDrive = hardwareMap.get(DcMotor.class, "motorRightFront");
            if (useGear20Motors) {
                backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                backRightDrive.setDirection(DcMotor.Direction.REVERSE);
                frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
                frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            } else {
                backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
                backRightDrive.setDirection(DcMotor.Direction.FORWARD);
                frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
                frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            }

            // initialize the encoder
            backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set all motors to zero power.
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
        }

        if (doArm) {
            arm = hardwareMap.get(DcMotor.class, "funkyShoulder");
            arm.setPower(0);
            armStart = arm.getCurrentPosition();
            armTarget = 0;

            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (doExtend) {
            extend = hardwareMap.get(DcMotor.class, "funkyShoulder2");
            extend.setPower(0);
            extendStart = extend.getCurrentPosition();
            extendTarget = 0;

            extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (doFinger) {
            finger = hardwareMap.get(Servo.class, "funkyClaw");
            wrist = hardwareMap.get(Servo.class, "funkyWrist");
        }

        if (doThePurplePixelPlopper) {
            thePurplePixelPlopper = hardwareMap.get(Servo.class, "pixelPlopper");
        }

        if (doGate) {
            // dont move!
            gate = hardwareMap.get(Servo.class, "gate");
        }

        initGyroscope();

       if (doObjectDetection) {
            initTfod();

            if (tfod != null) {
               // tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can increase the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 16/9).
               // tfod.setZoom(1.0, 16.0/9.0);
            } else {
                doObjectDetection = false;
            }
        }

        // Init run state.
        madeTheRun = false;
        startTime = 0;
        telemetry.addData("Init", "motors=%b, gyro=%b, objectDetect=%b", doMotors, doGyro, doObjectDetection);
        telemetry.update();
    }


    protected boolean initGyroscope() {
        if (doGyro) {
            imu = hardwareMap.get(IMU.class, "imu0");
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            return true;
        } else {
            return true;
        }
    }


    /**
     * @param deltaAngle must be between 0 and 359.9
     */
    protected void turnRight(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        float currentAngle = (float) getGyroscopeAngle();
        float destinationAngle = currentAngle - deltaAngle;
        turnRightAbsolute(destinationAngle);
    }

    /**
     * @param deltaAngle
     */
    protected void turnLeft(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        float currentAngle = (float) getGyroscopeAngle();
        float destinationAngle = currentAngle + deltaAngle;
        turnLeftAbsolute(destinationAngle);
    }

    protected void  turnRightAbsolute(float destinationAngle) {
        destinationAngle = normalizeAngle(destinationAngle); // between 0.0-359.999
        float currentAngle = (float) getGyroscopeAngle(); // between 0.0-359.999

        // destinationDiffAngle is going to tbe the number of degrees we still need to turn. Note tht if we have to transition over
        // the 360->0 boundary, then this number will be negative.
        float destinationDiffAngle = (destinationAngle - currentAngle);
        float diffRight = calculateRightDiff(destinationDiffAngle);
        float diffLeft = calculateLeftDiff(destinationDiffAngle);

        // Init debug info for printStatus().
        motorTurnType = "right";
        motorTurnDestination = destinationAngle;
        motorTurnAngleToGo = destinationDiffAngle;
        motorTurnAngleAdjustedToGo = diffRight;

        // we continue in this loop as long as we still need to transition over the 360->0 boundary, or until we are within NUDGE_ANGLE degrees of the target.
        while (opModeIsActive() && (diffLeft > NUDGE_ANGLE) && (diffRight > NUDGE_ANGLE)) {
            float oldAngle = currentAngle;

            double power = MOTOR_TURN_SPEED;
            if (diffRight < (NUDGE_ANGLE * 5))
                power *= 0.5;
            else if (diffRight < (NUDGE_ANGLE * 4))
                power *= 0.25;
            nudgeRight(power);

            currentAngle = (float) getGyroscopeAngle();
            destinationDiffAngle = (destinationAngle - currentAngle);
            diffRight = calculateRightDiff(destinationDiffAngle);
            diffLeft = calculateLeftDiff(destinationDiffAngle);
            motorTurnAngleToGo = destinationDiffAngle;
            motorTurnAngleAdjustedToGo = diffRight;
        }

        // Turn off the motor.
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        // Turn off debug info.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
    }

    protected void  turnLeftAbsolute(float destinationAngle) {
        destinationAngle = normalizeAngle(destinationAngle); // between 0.0-359.999
        float currentAngle = (float) getGyroscopeAngle(); // between 0.0-359.999

        // destinationDiffAngle is going to tbe the number of degrees we still need to turn. Note tht if we have to transition over
        // the 360->0 boundary, then this number will be negative.
        float destinationDiffAngle = (destinationAngle - currentAngle);
        float diffRight = calculateRightDiff(destinationDiffAngle);
        float diffLeft = calculateLeftDiff(destinationDiffAngle);

        // Init debug info for printStatus().
        motorTurnType = "left";
        motorTurnDestination = destinationAngle;
        motorTurnAngleToGo = destinationDiffAngle;
        motorTurnAngleAdjustedToGo = diffLeft;

        // we continue in this loop as long as we still need to transition over the 360->0 boundary, or until we are within NUDGE_ANGLE degrees of the target.
        while (opModeIsActive() && (diffLeft > NUDGE_ANGLE) && (diffRight > NUDGE_ANGLE)) {
            float oldAngle = currentAngle;

            double power = MOTOR_TURN_SPEED;
            if (diffLeft < (NUDGE_ANGLE * 5))
                power *= 0.5;
            else if (diffLeft < (NUDGE_ANGLE * 4))
                power *= 0.25;
            nudgeLeft(power);

            currentAngle = (float) getGyroscopeAngle();
            destinationDiffAngle = (destinationAngle - currentAngle);
            diffRight = calculateRightDiff(destinationDiffAngle);
            diffLeft = calculateLeftDiff(destinationDiffAngle);
            motorTurnAngleToGo = destinationDiffAngle;
            motorTurnAngleAdjustedToGo = diffLeft;
        }

        // Turn off the motor.
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);

        // Turn off debug info.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
    }

    protected void nudgeLeft(double power) {
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
     }

    protected void nudgeRight(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
    }

    // Always returns a number from 0-359.9999
    protected double getGyroscopeAngle() {
        if (doGyro) {
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            double calculated = orientation.getYaw(AngleUnit.DEGREES);
            //telemetry.addData("angle2","calculated:" + calculated);
            return normalizeAngle(calculated);
        } else {
            return 0.0f;
        }
    }

    public static float normalizeAngle(float inputAngle) {
        return (inputAngle + 360.0f) % 360.0f;
    }

    public static double normalizeAngle(double inputAngle) {
        return (inputAngle + 360.0) % 360.0;
    }

    public static float reverseAngle(float inputAngle) {
        return (inputAngle * -1.0f);
    }

    private float calculateRightDiff(float measuredDiff) {
        float ret = measuredDiff;
        while (ret < 0) {
            ret += 360.0;
        }
        return ret;
    }

    private float calculateLeftDiff(float measuredDiff) {
        float ret = measuredDiff;
        while (ret > 0) {
            ret -= 360.0;
        }
        return -ret;
    }

    float getAngleDifference(float from, float to) {
        float difference = to - from;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
    }

    protected void printStatus() {

        if (doGyro) {
        //    telemetry.addData("Gyro", "angle=%.1f", this.getGyroscopeAngle());
        } else {
            telemetry.addData("Gyro", "DISABLED");
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Run", "madetheRun=%b, runTime: %.1f, stopped=%b", madeTheRun, getRuntime(), isStopRequested());

        if (doMotors) {
            int backLeftPos = backLeftDrive.getCurrentPosition();
            int backRightPos = backRightDrive.getCurrentPosition();
            int frontLeftPos = frontLeftDrive.getCurrentPosition();
            int frontRightPos = frontRightDrive.getCurrentPosition();
            double backLeftPower = backLeftDrive.getPower();
            double backRightPower = backRightDrive.getPower();
            double frontLeftPower = frontLeftDrive.getPower();
            double frontRightPower = frontRightDrive.getPower();

            telemetry.addData("MotorDrive", "driving=%b, off=%02.1f, corr=%02.1f",
                    isDriving, driveAngleOffset, driveAngleCorrection);
           // telemetry.addData("MotorTurn", "type=%s, now: %02.1f, dest: %02.1f, togo=%02.1f, togo2=%02.1f",
            //        motorTurnType, this.getGyroscopeAngle(), motorTurnDestination, motorTurnAngleToGo, motorTurnAngleAdjustedToGo);
            telemetry.addData("MotorLeftBack", "start=%d, curr=%d, end=%d, pwr=%02.1f (%02.1f)",
                    driveBackLeftStart, backLeftPos, driveBackLeftTarget, driveLeftSpeed, backLeftPower);
            telemetry.addData("MotorRightBack", "start=%d, curr=%d, end=%d, pwr=%02.1f (%02.1f)",
                    driveBackRightStart, backRightPos, driveBackRightTarget, driveRightSpeed, backRightPower);
            telemetry.addData("MotorLefFront", "start=%d, curr=%d, end=%d, pwr=%02.1f (%02.1f)",
                    driveFrontLeftStart, frontLeftPos, driveFrontLeftTarget, driveLeftSpeed, frontLeftPower);
            telemetry.addData("MotorRightFront", "start=%d, curr=%d, end=%d, pwr=%02.1f (%02.1f)",
                    driveFrontRightStart, frontRightPos, driveFrontRightTarget, driveRightSpeed, frontRightPower);
        } else {
            telemetry.addData("Motor", "DISABLED");
        }

        if (doArm) {
            telemetry.addData("Arm", "start=%d, curr=%d, end=%d, pwr=%02.1f",
                 armStart, arm.getCurrentPosition(), armTarget, arm.getPower());
        }

        if (doExtend) {
            telemetry.addData("extend", "start=%d, curr=%d, end=%d, pwr=%02.1f",
                    extendStart, extend.getCurrentPosition(), extendTarget, extend.getPower());
        }

        if (doObjectDetection) {
            telemetry.addData("odd", "recogs=%d", recognitionsList.size());
            if (recognitionsList.size() > 0) {
                Recognition recognition = recognitionsList.get(0);
                telemetry.addData(String.format("odd-label"), recognition.getLabel());
                telemetry.addData(String.format("odd-left,top"), "%.03f , %.03f",
                        recognition.getLeft(), recognition.getTop());
                telemetry.addData(String.format("odd-right,bottom"), "%.03f , %.03f",
                        recognition.getRight(), recognition.getBottom());
            }
        } else {
            telemetry.addData("Tensor", "DISABLED");
        }

        telemetry.update();
    }

    protected void encoderDrive(double inches) {
        encoderDrive(inches, inches);
    }

    protected void encoderDrive(double leftInches, double rightInches) {
        double speed = MOTOR_MOVE_SPEED;
        encoderDrive(leftInches, rightInches, speed);
    }

    protected void encoderDrive(double leftInches, double rightInches, double speed) {
        encoderDrive(leftInches, rightInches, speed, getGyroscopeAngle());
    }

    protected void encoderDrive(double leftInches, double rightInches, double speed, double desiredAngle) {

        // Jump out if the motors are turned off.
        if (!doMotors)
            return;

        if (desiredAngle >= 0.0f) {
            motorPid.reset();
            motorPid.setSetpoint(0);
            motorPid.setOutputRange(0, speed);
            motorPid.setInputRange(-90, 90);
            motorPid.enable();
        }

        float startAngle = (float) getGyroscopeAngle();
        double countsPerMotor = COUNTS_PER_MOTOR_TORKNADO_60;
        if (useGear20Motors)
            countsPerMotor = COUNTS_PER_MOTOR_TORKNADO_20;
        double countsPerInch = countsPerMotor / (WHEEL_DIAMETER * Math.PI);
        int softStartDuration = 2000; // in milliseconds
        int brakeOffsetOne = (int) (18.0f * countsPerInch);
        int brakeOffsetTwo = (int) (8.0f * countsPerInch);

        // Get the starting position of the encoders.
        isDriving = true;
        driveBackLeftStart = backLeftDrive.getCurrentPosition();
        driveBackRightStart = backRightDrive.getCurrentPosition();
        driveFrontLeftStart = frontLeftDrive.getCurrentPosition();
        driveFrontRightStart = frontRightDrive.getCurrentPosition();

        int leftNew = (int) (leftInches * countsPerInch * RAT_FUDGE);
        int rightNew = (int) (rightInches * countsPerInch * RAT_FUDGE);
        driveBackLeftTarget = driveBackLeftStart + leftNew;
        driveBackRightTarget = driveBackRightStart + rightNew;
        driveFrontLeftTarget = driveFrontLeftStart + leftNew;
        driveFrontRightTarget = driveFrontRightStart + rightNew;
        backLeftDrive.setTargetPosition(driveBackLeftTarget);
        backRightDrive.setTargetPosition(driveBackRightTarget);
        frontLeftDrive.setTargetPosition(driveFrontLeftTarget);
        frontRightDrive.setTargetPosition(driveFrontRightTarget);

        // Turn On RUN_TO_POSITION
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Compute the braking zones (based on back wheels).
        int leftBrakeOne = driveBackLeftStart + brakeOffsetOne; // how many remaining will trigger it
        int rightBrakeOne = driveBackRightStart + brakeOffsetOne;
        int leftBrakeTwo = driveBackLeftStart + brakeOffsetTwo;
        int rightBrakeTwo = driveBackRightStart + brakeOffsetTwo;

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        ElapsedTime motorOnTime = new ElapsedTime();
        boolean keepGoing = true;
        while (opModeIsActive() && keepGoing && (motorOnTime.seconds() < 30)) {
            printStatus();

            int backLeftPos = backLeftDrive.getCurrentPosition();
            int backRightPos = backRightDrive.getCurrentPosition();

            // soft start
            double currSpeed = speed;
            double elapsed = motorOnTime.milliseconds();
            if (elapsed < softStartDuration) {
                double ratio = elapsed / softStartDuration;
                currSpeed *= ratio;
            }

            // Throttle speed down as we approach our target
            int remainingLeft = driveBackLeftTarget - backLeftPos;
            int remainingRight = driveBackRightTarget - backRightPos;
            if ((Math.abs(remainingLeft) < brakeOffsetTwo) || (Math.abs(remainingRight) < brakeOffsetTwo)) {
                currSpeed *= 0.25;
            } else if ((Math.abs(remainingLeft) < brakeOffsetOne) || (Math.abs(remainingRight) < brakeOffsetOne)) {
                currSpeed *= 0.5;
            }

            // Calculate PID correction = straighten out the line!
            if (desiredAngle >= 0.0f) {
                float currentAngle = (float) getGyroscopeAngle();
                driveAngleOffset = getAngleDifference(currentAngle, (float)desiredAngle);
                driveAngleCorrection = (float) motorPid.performPID(driveAngleOffset);
                if ((leftInches < 0) && (rightInches < 0)) {
                    driveAngleCorrection = -driveAngleCorrection;
                }
            }

            // Record and apply the desired power level.
            driveLeftSpeed = currSpeed + driveAngleCorrection;
            driveRightSpeed = currSpeed - driveAngleCorrection;
            backLeftDrive.setPower(Math.abs(driveLeftSpeed));
            backRightDrive.setPower(Math.abs(driveRightSpeed));
            frontLeftDrive.setPower(Math.abs(driveLeftSpeed));
            frontRightDrive.setPower(Math.abs(driveRightSpeed));

            keepGoing = backRightDrive.isBusy() && backLeftDrive.isBusy();
        }

        // Turn off RUN_TO_POSITION
        printStatus();
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackLeftStart = 0;
        driveBackRightStart = 0;
        driveFrontLeftStart = 0;
        driveFrontRightStart = 0;
        driveBackLeftTarget = 0;
        driveBackRightTarget = 0;
        driveFrontLeftTarget = 0;
        driveFrontRightTarget = 0;
        driveLeftSpeed = 0.0f;
        driveRightSpeed = 0.0f;
        driveAngleOffset = 0.0f;
        driveAngleCorrection = 0.0f;
        isDriving = false;
    }

    protected void fullTiltForward(int seconds)
    {
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double power = 1.0;
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);

        ratCrewWaitMillis(seconds * 1000);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
       tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
               .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
       // builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /*
    public enum SignalConfig
    {
        UNDETERMINED,
        ONE,
        TWO,
        THREE,
    } */

    /*protected SignalConfig getSignalConfiguration()
    {
        SignalConfig config = SignalConfig.UNDETERMINED;
        if (recognitionsList.size() <= 0) {
            config = SignalConfig.UNDETERMINED;
        }
        else {
            Recognition r = recognitionsList.get(0);
            if (r.getLabel().equalsIgnoreCase("3 Panel")) {
                config = SignalConfig.THREE;
            } else if (r.getLabel().equalsIgnoreCase("2 Bulb")) {
                config = SignalConfig.TWO;
            } else if (r.getLabel().equalsIgnoreCase("1 Bolt")) {
                config = SignalConfig.ONE;
            } else {
                config = SignalConfig.UNDETERMINED;
            }
            }
        return config;
    } */


    protected List<Recognition> getRecognitions() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }

        return new ArrayList<>(currentRecognitions);
    }

    /*protected SignalConfig SignalIdentifier() {
        ratCrewWaitMillis(WAIT_TIME);

        if (doVuforia) {

            List<Recognition> updatedRecognitions = tfod.getRecognitions();
            if (updatedRecognitions != null) {
                // make a copy of the list
                recognitionsList = new ArrayList<>(updatedRecognitions);
            }
            telemetry.addData("# Object Detected", recognitionsList.size());
        }

        // step through the list of recognitions and display boundary info.
        int i = 0;

        for (Recognition recognition : recognitionsList) {
            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
        }


        SignalConfig rc = getSignalConfiguration();
        return rc;
    } */


    protected void ratCrewWaitSecs(long secs) {
        ratCrewWaitMillis(secs * 1000);
    }

    protected void ratCrewWaitMillis(long millis){
        long startTime = System.currentTimeMillis();
        long currentTime = startTime;
        long timeElapsed = currentTime - startTime;
        while (opModeIsActive() && (timeElapsed < millis)) {
            printStatus();

            sleep(100);

            currentTime = System.currentTimeMillis();
            timeElapsed = currentTime - startTime;
        }
    }




    protected void openGate() {
        if (doGate) {
            gate.setPosition(0.0);
        }
    }

    protected void closeGate() {
        if (doGate) {
            gate.setPosition(1.0);
        }
    }

    protected void openFinger() {
        if (doFinger) {
            finger.setPosition(1.0);
        }
    }
    protected void closeFinger() {
        if (doFinger) {
            finger.setPosition(0.0);
        }
    }


    protected void raiseWrist(int i) {
        if (doFinger) {
            wrist.setPosition(1.0);
        }
    }
    protected void lowerWrist(int i) {
        if (doFinger) {
            wrist.setPosition(0.0);
        }
    }
    protected void wristSet(float i) {
        if (doFinger) {
            wrist.setPosition(i);
        }
    }

    protected void plopThePurplePixel() {
        if(doThePurplePixelPlopper) {
            thePurplePixelPlopper.setPosition(0.1);
            ratCrewWaitSecs(2);
            thePurplePixelPlopper.setPosition(1.0);
            ratCrewWaitSecs(2);
        }
    }

    protected void nudgeArmUp() {
        moveArmUp(500);
    }

    protected void moveArmUp(int mill) {
        if (doArm) {
            arm.setPower(-0.7);
            ratCrewWaitMillis(mill);
            arm.setPower(0);
        }
    }

    protected void moveArmDown(int mill) {
        if (doArm) {
            arm.setPower(0.7);
            ratCrewWaitMillis(mill);
            arm.setPower(0);
        }
    }

    protected void moveArmCounter(int counter) {
        moveArmCounter(counter, 0.25f);
    }

    protected void moveArmCounter(int counter, float power) {
        if (doArm) {
            armTarget = armStart + counter;
            printStatus();

            // Tell the motor its end spot, and start it up.
            arm.setTargetPosition(armTarget);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Loop and wait until its done.
            ElapsedTime motorOnTime = new ElapsedTime();
            boolean keepGoing = true;
            while (opModeIsActive() && keepGoing && (motorOnTime.seconds() < 30)) {
                arm.setPower(power);
                printStatus();
                keepGoing = arm.isBusy();
            }

            // Clear out the state.
            printStatus();
            arm.setPower(0);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            armTarget = 0;
        }
    }

    protected void moveextendcounter(int counter, float power) {
        if (doExtend) {
            extendTarget = extendStart + counter;
            printStatus();

            // Tell the motor its end spot, and start it up.
            extend.setTargetPosition(extendTarget);
            extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Loop and wait until its done.
            ElapsedTime motorOnTime = new ElapsedTime();
            boolean keepGoing = true;
            while (opModeIsActive() && keepGoing && (motorOnTime.seconds() < 30)) {
                extend.setPower(power);
                printStatus();
                keepGoing = extend.isBusy();
            }

            // Clear out the state.
            printStatus();
            extend.setPower(0);
            extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendTarget = 0;
        }
    }
}
