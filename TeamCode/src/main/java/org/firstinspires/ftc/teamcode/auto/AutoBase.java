package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 */
public abstract class AutoBase extends OpMode {

    // MAGIC NUMBERS for the motor encoders
    // https://asset.pitsco.com/sharedimages/resources/torquenado_dcmotorspecifications.pdf
    static final float COUNTS_PER_MOTOR_TORKNADO = 1440;  // 24 cycles per revolution, times 60:1 geared down.
    // http://www.revrobotics.com/content/docs/Encoder-Guide.pdf
    static final float COUNTS_PER_MOTOR_REV_HDHEX_40  = 1120; // 28 cycles per rotation at the main motor, times 40:1 geared down
    static final float COUNTS_PER_MOTOR_REV_HDHEX_20 = 560; // 28 cycles per rotation at the main motor, times 20:1 geared down

    private static final int NUDGE_TIME = 1;
    private static final float NUDGE_ANGLE = 4.0f;
    private static final float NORMALIZE_ANGLE = 360.0f;
    private static final float MOTOR_TURN_SPEED = 0.6f;
    private static final float MOTOR_MOVE_SPEED = 0.8f;
    private static final float COUNTS_PER_MOTOR = COUNTS_PER_MOTOR_TORKNADO;
    private static final float WHEEL_DIAMETER = 4.0f;

    // Instance Members.
    private boolean doMotors = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private String motorTurnType = "none";
    private float motorTurnDestination = 0.0f;
    private float motorTurnAngleToGo = 0.0f;
    private float motorTurnAngleAdjustedToGo = 0.0f;

    private boolean doGyro = true;
    private BNO055IMU bosch;

    // run state
    private boolean madeTheRun = false;
    private boolean canceled = false;



    // Called once, right after hitting the Init button.
    @Override
    public void init() {

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
        }

        initGyroscope();

        // Init run state.
        madeTheRun = false;
        canceled = false;

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
    }

    protected boolean initGyroscope() {
        if (doGyro) {
            bosch = hardwareMap.get(BNO055IMU.class, "imu0");
            telemetry.addData("Gyro", "class:" + bosch.getClass().getName());

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
    public void stop() {
        canceled = true;
    }

    //all Opmodes must override
    public abstract void ratCrewGo();

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        if (madeTheRun == false) {

            ratCrewGo();
            madeTheRun = true;
        }

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
        while ((diffLeft > NUDGE_ANGLE) && (diffRight > NUDGE_ANGLE)) {
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
        while ((diffLeft > NUDGE_ANGLE) && (diffRight > NUDGE_ANGLE)) {
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

    protected void sleep(long milliseconds) {
        try {
            ElapsedTime sleepTime = new ElapsedTime();
            while (!canceled && sleepTime.milliseconds() < milliseconds) {
                Thread.sleep(1);
                printStatus();
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     *
     */
    protected void printStatus() {

        if (doGyro) {
            telemetry.addData("Gyro", "angle=%.2f", this.getGyroscopeAngle());
        } else {
            telemetry.addData("Gyro", "DISABLED");
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Gyro", "angle: " + this.getGyroscopeAngle());
        telemetry.addData("Run", "madeTheRun=%b", madeTheRun);
        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());

        if (doMotors) {
           /* telemetry.addData("Motor: frnt", "left:%02.1f, (%d), rigt: %02.1f, (%d)",
                    motorFrontLeft.getPower(), motorFrontLeft.getCurrentPosition(), motorFrontRight.getPower(), motorFrontRight.getCurrentPosition());
            telemetry.addData("Motor: back", "left:%02.1f, (%d), rigt: %02.1f, (%d)",
                    motorBackLeft.getPower(), motorBackLeft.getCurrentPosition(), motorBackRight.getPower(), motorBackRight.getCurrentPosition());
            telemetry.addData("Motor: turn", "type=%s, now: %02.1f, dest: %02.1f, togo= %02.1f, togo2= %02.1f",
                    motorTurnType, this.getGyroscopeAngle(), motorTurnDestination, motorTurnAngleToGo, motorTurnAngleAdjustedToGo); */
        } else {
            telemetry.addData("Motor", "DISABLED");
        }


           /* if (useVuforia) {
                int numStones = 0;
                int numSkyStones = 0;

                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = getRecognitions();
                    if (updatedRecognitions == null) {
                        numStones = lastStones;
                        numSkyStones = lastSkyStones;
                    } else {
                        numStones = updatedRecognitions.size();
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel() == "Skystone") {
                                numSkyStones++;
                                numStones--;
                            }
                        }
                        lastStones = numStones;
                        lastSkyStones = numSkyStones;
                    }
                }

                telemetry.addData("StoneDetect", "mode:%s, norm: %d, sky: %d, loc: %02.1f,  %02.1f,  %02.1f", stoneconfig, numStones, numSkyStones,
                        leftEdgeSkyStone, rightEdgeSkyStone, widthSkyStone);
            } else {
                telemetry.addData("StoneDetect", "DISABLED");
            } */

        telemetry.addData("Status", "ran=%b, cncl=%b, time=%.2f", madeTheRun, canceled, getRuntime());
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

        float startAngle = getGyroscopeAngle();

        // Jump out if the motors are turned off.
        if (!doMotors)
            return;

        double countsPerInch = COUNTS_PER_MOTOR / (WHEEL_DIAMETER * Math.PI);

        // Get the current position.
        int leftBackStart = leftDrive.getCurrentPosition();
        int rightBackStart = rightDrive.getCurrentPosition();
        int leftFrontStart = 0;
        int rightFrontStart = 0;
        telemetry.addData("encoderDrive", "Starting %7d, %7d, %7d, %7d",
                leftBackStart, rightBackStart, leftFrontStart, rightFrontStart);

        // Determine new target position, and pass to motor controller
        int leftBackTarget = leftBackStart + (int) (leftInches * countsPerInch);
        int rightBackTarget = rightBackStart + (int) (rightInches * countsPerInch);

        leftDrive.setTargetPosition(leftBackTarget);
        rightDrive.setTargetPosition(rightBackTarget);
        telemetry.addData("encoderDrive", "Target %7d, %7d",
                leftBackTarget, rightBackTarget);

        // Throttle speed down as we approach our target
        if ((Math.abs(leftInches) < 8.0) || (Math.abs(rightInches) < 8.0)) {
            speed *= 0.5;
        } else  if ((Math.abs(leftInches) < 5.0) || (Math.abs(rightInches) < 5.0)) {
            speed *= 0.25;
        }

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        ElapsedTime motorOnTime = new ElapsedTime();
        boolean keepGoing = true;
        while (keepGoing && (motorOnTime.seconds() < 30)) {


                telemetry.addData("encoderDrive1", "Running at %7d, %7d",
                        leftDrive.getCurrentPosition(),
                        rightDrive.getCurrentPosition());
                telemetry.addData("encoderDrive2", "Running to %7d, %7d",
                        leftBackTarget,
                        rightBackTarget);
                keepGoing = rightDrive.isBusy() && leftDrive.isBusy();

            if (keepGoing) {
                // Calculate PID correction = straightne out the line!
                double correction = 0;

                leftDrive.setPower(Math.abs(speed - correction));
                rightDrive.setPower(Math.abs(speed + correction));
            }

            // telemetry.update();
            //sleep(100);
        }

        // Turn off RUN_TO_POSITION
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*telemetry.addData("encoderDrive", "Finished (%s) at %7d,%7d,%7d,%7d to [%7d,%7d,%7d,%7d] (%7d,%7d,%7d,%7d)",
                motorOnTime.toString(),
                leftBackStart,
                rightBackStart,
                leftFrontStart,
                rightFrontStart,
                motorBackLeft.getCurrentPosition(),
                motorBackRight.getCurrentPosition(),
                motorFrontLeft.getCurrentPosition(),
                motorFrontRight.getCurrentPosition(),
                leftBackTarget,
                rightBackTarget,
                leftFrontTarget,
                rightFrontTarget); */
        // sleep(100);
    }

}