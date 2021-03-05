package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.barebones.BareBones_RingDetection;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 */
public abstract class AutoBase extends LinearOpMode {

    public enum RingConfig
    {
        UNDETERMINED,
        EMPTY,
        SINGLE,
        QUAD,
    }

    // MAGIC NUMBERS for the motor encoders
    // https://asset.pitsco.com/sharedimages/resources/torquenado_dcmotorspecifications.pdf
    static final float COUNTS_PER_MOTOR_TORKNADO = 1440;  // 24 cycles per revolution, times 60:1 geared down.
    // http://www.revrobotics.com/content/docs/Encoder-Guide.pdf
    static final float COUNTS_PER_MOTOR_REV_HDHEX_40  = 1120; // 28 cycles per rotation at the main motor, times 40:1 geared down
    static final float COUNTS_PER_MOTOR_REV_HDHEX_20 = 560; // 28 cycles per rotation at the main motor, times 20:1 geared down

    // Vuforia Constants
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY =
            "AfgOBrf/////AAABmRjMx12ilksPnWUyiHDtfRE42LuceBSFlCTIKmmNqCn2EOk3I4NtDCSr0wCLFxWPoLR2qHKraX49ofQ2JknI76SJS5Hy8cLbIN+1GlFDqC8ilhuf/Y1yDzKN6a4n0fYWcEPlzHRc8C1V+D8vZ9QjoF3r//FDDtm+M3qlmwA7J/jNy4nMSXWHPCn2IUASoNqybTi/CEpVQ+jEBOBjtqxNgb1CEdkFJrYGowUZRP0z90+Sew2cp1DJePT4YrAnhhMBOSCURgcyW3q6Pl10XTjwB4/VTjF7TOwboQ5VbUq0wO3teE2TXQAI53dF3ZUle2STjRH0Rk8H94VtHm9u4uitopFR7zmxVl3kQB565EUHwfvG";

    private static final int NUDGE_TIME = 1;
    private static final float NUDGE_ANGLE = 4.0f;
    private static final float NORMALIZE_ANGLE = 360.0f;
    private static final float MOTOR_TURN_SPEED = 0.6f;
    private static final float MOTOR_MOVE_SPEED = 0.8f;
    private static final float COUNTS_PER_MOTOR = COUNTS_PER_MOTOR_TORKNADO;
    private static final float WHEEL_DIAMETER = 4.0f;
    private static final long WAIT_TIME = TimeUnit.SECONDS.toMillis(5L);
    private static final float RAT_FUDGE = 0.98f;

    // Instance Members: Motors
    private boolean doMotors = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Servo gate;
    private String motorTurnType = "none";
    private float motorTurnDestination = 0.0f;
    private float motorTurnAngleToGo = 0.0f;
    private float motorTurnAngleAdjustedToGo = 0.0f;
    private boolean isDriving = false;
    private float driveAngleOffset = 0.0f;
    private float driveAngleCorrection = 0.0f;
    private int driveLeftStart = 0;
    private int driveRightStart = 0;
    private int driveLeftTarget = 0;
    private int driveRightTarget = 0;
    private double driveRightSpeed = 0.0f;
    private double driveLeftSpeed = 0.0f;
    private PIDController motorPid = new PIDController(.05, 0, 0);


    // Instance Members: Gyro
    private boolean doGyro = true;
    private BNO055IMU bosch;

    // Instance Members: Core
    private long startTime =  System.nanoTime();
    private boolean madeTheRun = false;

    // Instance Members: Vuforia
    private boolean doVuforia = true;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private List<Recognition> recognitionsList = new ArrayList<>();


    // Called once, right after hitting the Init button.
    protected void ratCrewInit() {

        // Init motor state machine.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
        isDriving = false;
        driveLeftStart = 0;
        driveRightStart = 0;
        driveLeftTarget = 0;
        driveRightTarget = 0;
        driveRightSpeed = 0.0f;
        driveLeftSpeed = 0.0f;
        driveAngleOffset = 0.0f;
        driveAngleCorrection = 0.0f;

        // TODO: push into initMotor function
        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);

            // initialize the encoder
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Set all motors to zero power.
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            gate = hardwareMap.get(Servo.class, "Gate");
            gate.setPosition(1.0);
        }
        initGyroscope();

        if (doVuforia){
            initVuforia();
            initTfod();

            if (tfod != null) {
                tfod.activate();

                // The TensorFlow software will scale the input images from the camera to a lower resolution.
                // This can result in lower detection accuracy at longer distances (> 55cm or 22").
                // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
                // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
                // should be set to the value of the images used to create the TensorFlow Object Detection model
                // (typically 1.78 or 16/9).

                // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
                tfod.setZoom(2.5, 1.78);
            }
        }

        // Init run state.
        madeTheRun = false;
        startTime = 0;
        telemetry.addData("Init", "motors=%b, gyro=%b, vuforia=%b", doMotors, doGyro, doVuforia);
        telemetry.update();
    }


    protected boolean initGyroscope() {
        if (doGyro) {
            bosch = hardwareMap.get(BNO055IMU.class, "imu0");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;
            parameters.loggingTag = "bosch";
            boolean boschInit = bosch.initialize(parameters);
            return boschInit;
        } else {
            return true;
        }
    }

    @Override
    public void runOpMode(){
        ratCrewInit();
        waitForStart();

        printStatus();
        ratCrewStart();
        printStatus();
    }

    //all Opmodes must override
    public abstract void ratCrewGo();

    // Called repeatedly, right after hitting start, up until hitting stop.
    public void ratCrewStart() {
        printStatus();
        ratCrewGo();
        madeTheRun = true;
        printStatus();
    }

    /**
     * @param deltaAngle must be between 0 and 359.9
     */
    protected void turnRight(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        float currentAngle = getGyroscopeAngle();
        float destinationAngle = currentAngle + deltaAngle;
        turnRightAbsolute(destinationAngle);
    }

    /**
     * @param deltaAngle
     */
    protected void turnLeft(float deltaAngle) {
        assert (deltaAngle > 0.0);
        assert (deltaAngle <= 360.0);

        float currentAngle = getGyroscopeAngle();
        float destinationAngle = currentAngle - deltaAngle;
        turnLeftAbsolute(destinationAngle);
    }

    protected void  turnRightAbsolute(float destinationAngle) {
        destinationAngle = normalizeAngle(destinationAngle); // between 0.0-359.999
        float currentAngle = getGyroscopeAngle(); // between 0.0-359.999

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

            currentAngle = getGyroscopeAngle();
            destinationDiffAngle = (destinationAngle - currentAngle);
            diffRight = calculateRightDiff(destinationDiffAngle);
            diffLeft = calculateLeftDiff(destinationDiffAngle);
            motorTurnAngleToGo = destinationDiffAngle;
            motorTurnAngleAdjustedToGo = diffRight;
        }

        // Turn off the motor.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off debug info.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
    }

    protected void  turnLeftAbsolute(float destinationAngle) {
        destinationAngle = normalizeAngle(destinationAngle); // between 0.0-359.999
        float currentAngle = getGyroscopeAngle(); // between 0.0-359.999

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

            currentAngle = getGyroscopeAngle();
            destinationDiffAngle = (destinationAngle - currentAngle);
            diffRight = calculateRightDiff(destinationDiffAngle);
            diffLeft = calculateLeftDiff(destinationDiffAngle);
            motorTurnAngleToGo = destinationDiffAngle;
            motorTurnAngleAdjustedToGo = diffLeft;
        }

        // Turn off the motor.
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off debug info.
        motorTurnType = "none";
        motorTurnDestination = 0.0f;
        motorTurnAngleToGo = 0.0f;
        motorTurnAngleAdjustedToGo = 0.0f;
    }

    protected void nudgeLeft(double power) {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
     }

    protected void nudgeRight(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
    }

    // Always returns a number from 0-359.9999
    protected float getGyroscopeAngle() {
        if (doGyro) {
            Orientation exangles = bosch.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            float gyroAngle = exangles.thirdAngle;
            //exangles.
            //telemetry.addData("angle", "angle: " + exangles.thirdAngle);
            float calculated = normalizeAngle(reverseAngle(gyroAngle));
            //telemetry.addData("angle2","calculated:" + calculated);
            return calculated;
        } else {
            return 0.0f;
        }
    }

    public static float normalizeAngle(float inputAngle) {
        return (inputAngle + 360.0f) % 360.0f;
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

    protected void printStatus() {

        if (doGyro) {
            telemetry.addData("Gyro", "angle=%.1f", this.getGyroscopeAngle());
        } else {
            telemetry.addData("Gyro", "DISABLED");
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Run", "madetheRun=%b, runTime: %.1f, stopped=%b", madeTheRun, getRuntime(), isStopRequested());

        if (doMotors) {
            int leftPos = leftDrive.getCurrentPosition();
            int rightPos = rightDrive.getCurrentPosition();

            telemetry.addData("Motor", "left: %02.1f (%d), right: %02.1f (%d), driving=%b, off=%02.1f, corr=%02.1f",
                    leftDrive.getPower(), leftPos, rightDrive.getPower(), rightPos, isDriving, driveAngleOffset, driveAngleCorrection);
            telemetry.addData("MotorTurn", "type=%s, now: %02.1f, dest: %02.1f, togo=%02.1f, togo2=%02.1f",
                    motorTurnType, this.getGyroscopeAngle(), motorTurnDestination, motorTurnAngleToGo, motorTurnAngleAdjustedToGo);
            telemetry.addData("MotorLeft", "start=%d, curr=%d, end=%d, pwr=%02.1f", driveLeftStart, leftPos, driveLeftTarget, driveLeftSpeed);
            telemetry.addData("MotorRight", "start=%d, curr=%d, end=%d, pwr=%02.1f", driveRightStart, rightPos, driveRightTarget, driveRightSpeed);
        } else {
            telemetry.addData("Motor", "DISABLED");
        }

        if (doVuforia) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                  recognitionsList = new ArrayList<>(updatedRecognitions);
                }
            }
            telemetry.addData("Tensor", "config=%s, recogs=%d", getRingConfiguration(), recognitionsList.size());
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

        float startAngle = getGyroscopeAngle();
        double countsPerInch = COUNTS_PER_MOTOR / (WHEEL_DIAMETER * Math.PI);
        int softStartDuration = 2000; // in milliseconds
        int brakeOffsetOne = (int) (18.0f * countsPerInch);
        int brakeOffsetTwo = (int) (8.0f * countsPerInch);

        // Get the starting position of the encoders.
        isDriving = true;
        driveLeftStart = leftDrive.getCurrentPosition();
        driveRightStart = rightDrive.getCurrentPosition();

        int leftNew = (int) (leftInches * countsPerInch * RAT_FUDGE);
        int rightNew = (int) (rightInches * countsPerInch * RAT_FUDGE);
        driveLeftTarget = driveLeftStart + leftNew;
        driveRightTarget = driveRightStart + rightNew;
        leftDrive.setTargetPosition(driveLeftTarget);
        rightDrive.setTargetPosition(driveRightTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Compute the braking zones.
        int leftBrakeOne = driveLeftStart + brakeOffsetOne; // how many remaining will trigger it
        int rightBrakeOne = driveRightStart + brakeOffsetOne;
        int leftBrakeTwo = driveLeftStart + brakeOffsetTwo;
        int rightBrakeTwo = driveRightStart + brakeOffsetTwo;


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

            int leftPos = leftDrive.getCurrentPosition();
            int rightPos = rightDrive.getCurrentPosition();

            // soft start
            double currSpeed = speed;
            double elapsed = motorOnTime.milliseconds();
            if (elapsed < softStartDuration) {
                double ratio = elapsed / softStartDuration;
                currSpeed *= ratio;
            }

            // Throttle speed down as we approach our target
            int remainingLeft = driveLeftTarget - leftPos;
            int remainingRight = driveRightTarget - rightPos;
            if ((Math.abs(remainingLeft) < brakeOffsetTwo) || (Math.abs(remainingRight) < brakeOffsetTwo)) {
                currSpeed *= 0.25;
            } else if ((Math.abs(remainingLeft) < brakeOffsetOne) || (Math.abs(remainingRight) < brakeOffsetOne)) {
                currSpeed *= 0.5;
            }

            // Calculate PID correction = straighten out the line!
            if (desiredAngle >= 0.0f) {
                float currentAngle = getGyroscopeAngle();
                driveAngleOffset = getAngleDifference(currentAngle, (float)desiredAngle);
                driveAngleCorrection = (float) motorPid.performPID(driveAngleOffset);
                if ((leftInches < 0) && (rightInches < 0)) {
                    driveAngleCorrection = -driveAngleCorrection;
                }
            }

            // Record and apply the desired power level.
            driveLeftSpeed = currSpeed - driveAngleCorrection;
            driveRightSpeed = currSpeed + driveAngleCorrection;
            leftDrive.setPower(Math.abs(driveLeftSpeed));
            rightDrive.setPower(Math.abs(driveRightSpeed));

            keepGoing = rightDrive.isBusy() && leftDrive.isBusy();
        }

        // Turn off RUN_TO_POSITION
        printStatus();
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveLeftStart = 0;
        driveRightStart = 0;
        driveLeftTarget = 0;
        driveRightTarget = 0;
        driveLeftSpeed = 0.0f;
        driveRightSpeed = 0.0f;
        driveAngleOffset = 0.0f;
        driveAngleCorrection = 0.0f;
        isDriving = false;
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    protected RingConfig getRingConfiguration()
    {
        RingConfig currentRings = RingConfig.UNDETERMINED;
        if (recognitionsList.size() <= 0) {
            currentRings = RingConfig.EMPTY;
        }
        else if (recognitionsList.size() > 1) {
            currentRings = RingConfig.UNDETERMINED;
        }
        else {
            Recognition r = recognitionsList.get(0);
            if (r.getLabel().equalsIgnoreCase("quad")){
                currentRings = RingConfig.QUAD;
            } else {
                currentRings = RingConfig.SINGLE;
            }
        }
        return currentRings;
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

    protected RingConfig ringIdentifier() {
        ratCrewWaitMillis(WAIT_TIME);

        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            // make a copy of the list
            recognitionsList = new ArrayList<>(updatedRecognitions);
        }
        telemetry.addData("# Object Detected", recognitionsList.size());

        // step through the list of recognitions and display boundary info.
        /*int i = 0;

        for (Recognition recognition : recognitionsList) {
            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
        }
        */

        RingConfig rc = getRingConfiguration();
        telemetry.addData("RingIdent", "recogs=%d, config=%s", recognitionsList.size(), rc);
        telemetry.update();
        return rc;
    }

    protected void openGate() {
        gate.setPosition(1.0);
    }

    protected void closeGate() {
        gate.setPosition(0.0);
    }

    float getAngleDifference(float from, float to)
    {
        float difference = to - from;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
    }


}
