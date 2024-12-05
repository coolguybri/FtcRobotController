package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame Deep!", group="AAA")
public class Teleop_IntoTheDeep extends OpMode {

    // Throttler
    private enum ThrottlerGear {
        GEAR_TWO,
        GEAR_ONE,
    };
    private boolean throttlerButtonLast = false;
    private ThrottlerGear throttlerGear = ThrottlerGear.GEAR_ONE;
    private boolean armThrottlerButtonLast = false;
    private ThrottlerGear armThrottlerGear = ThrottlerGear.GEAR_ONE;

    // 4-wheel drive
    private boolean doMotors = true;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;

    // FunkyArm
    private boolean doExtendoArm = true;
    private Servo funkyClaw;
    private boolean funkyClawLock = false;
    private boolean funkyClawButtonLast = false;
    private Servo funkyWrist;
    private DcMotor funkyShoulder;
    private int funkyShoulderStartPos = 0;
    private int funkyShoulderEndPos = 0;

    private DcMotor funkyExtender;
    private int funkyExtenderStartPos = 0;
    private int funkyExtenderEndPos = 0;

    // sample
    private Servo sampleSlopper;


    // Called once, right after hitting the Init button.
    @Override
    public void init() {

        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            frontLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftFront");
            frontRightDrive = hardwareMap.get(DcMotor.class, "motorRightFront");
            backLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftRear");
            backRightDrive = hardwareMap.get(DcMotor.class, "motorRightRear");

            // This changes based on the gear box.
            //frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            //frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            //backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            //backRightDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power.
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Set all motors to run without encoders; manual control.
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        sampleSlopper = hardwareMap.get(Servo.class, "pixelPlopper");

        boolean errFunkyArm = false;
        if (doExtendoArm) {
            try {
                // claw
                funkyClawButtonLast = false;
                funkyClawLock = true;
                funkyClaw = hardwareMap.get(Servo.class, "funkyClaw");

                // wrist
                funkyWrist = hardwareMap.get(Servo.class, "funkyWrist");
                funkyWrist.setPosition(0.5f);

                 // shoulder
                funkyShoulder = hardwareMap.get(DcMotor.class, "funkyShoulder");
                funkyShoulder.setDirection(DcMotor.Direction.REVERSE);
                funkyShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyShoulder.setPower(0);
                funkyShoulderStartPos = funkyShoulder.getCurrentPosition();
                funkyShoulderEndPos = funkyShoulderStartPos + 200;

                // Extendo
                funkyExtender = hardwareMap.get(DcMotor.class, "funkyShoulder2");
                funkyExtender.setDirection(DcMotor.Direction.REVERSE);
                funkyExtender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyExtender.setPower(0);
                funkyExtenderStartPos = funkyExtender.getCurrentPosition();
                funkyExtenderEndPos = funkyExtenderStartPos + 200;

            } catch (Exception e) {
                doExtendoArm = false;
                errFunkyArm = true;
                telemetry.addData("Error", "ExtendoArm: %s", e.getMessage());
            }
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized ExtendoArm, do=%s, err=%s", doExtendoArm, errFunkyArm);
    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        // Motor Throttler: X button
        boolean throttlerButtonNow = gamepad1.x;
        if (!throttlerButtonLast && throttlerButtonNow) {
            switch (throttlerGear) {
                case GEAR_TWO:
                    throttlerGear = ThrottlerGear.GEAR_ONE;
                    break;
                case GEAR_ONE:
                    throttlerGear = ThrottlerGear.GEAR_TWO;
                    break;
            }
        }
        throttlerButtonLast = throttlerButtonNow;

        // Arm throttler: Left Bumper
        /*boolean armThrottlerButtonNow = gamepad1.left_bumper;
        if (!armThrottlerButtonLast && armThrottlerButtonNow) {
            switch (armThrottlerGear) {
            case GEAR_TWO:
                armThrottlerGear = ThrottlerGear.GEAR_ONE;
                break;
            case GEAR_ONE:
                armThrottlerGear = ThrottlerGear.GEAR_TWO;
                break;
            }
        }
        armThrottlerButtonLast = armThrottlerButtonNow;  */

        if (doMotors) {
            double leftBackPower = 0.0;
            double rightBackPower = 0.0;
            double leftFrontPower = 0.0;
            double rightFrontPower = 0.0;

            double motorScaler = 1.0;
            switch (throttlerGear) {
            case GEAR_TWO:
                motorScaler = 0.75;
                break;
            case GEAR_ONE:
                motorScaler = 0.4;
                break;
            }

            //keep drive the same, turn--switch to right joystic
            double drive = -gamepad1.left_stick_y;
            if (Math.abs(drive) < 0.1) {
                drive = -gamepad1.right_stick_y;
            }
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            if (Math.abs(turn) > 0.1) {
                //leftBackPower = Range.clip(drive + turn, -1.0, 1.0);
                //leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
                //rightBackPower = Range.clip(drive - turn, -1.0, 1.0);
                //rightFrontPower = Range.clip(drive - turn, -1.0, 1.0);
                leftBackPower = Range.clip(drive - turn, -1.0, 1.0);
                leftFrontPower = Range.clip(drive - turn, -1.0, 1.0);
                rightBackPower = Range.clip(drive + turn, -1.0, 1.0);
                rightFrontPower = Range.clip(drive + turn, -1.0, 1.0);
            } else {
                leftBackPower = Range.clip(drive - strafe, -1.0, 1.0);
                leftFrontPower = Range.clip(drive + strafe, -1.0, 1.0);
                rightBackPower = Range.clip(drive + strafe, -1.0, 1.0);
                rightFrontPower = Range.clip(drive - strafe, -1.0, 1.0);
                //leftBackPower = Range.clip(drive + strafe, -1.0, 1.0);
                //leftFrontPower = Range.clip(drive - strafe, -1.0, 1.0);
                //rightBackPower = Range.clip(drive - strafe, -1.0, 1.0);
                //rightFrontPower = Range.clip(drive + strafe, -1.0, 1.0);
            }

            // Send calculated power to wheels
            backLeftDrive.setPower(leftBackPower * motorScaler);
            backRightDrive.setPower(rightBackPower * motorScaler);
            frontLeftDrive.setPower(leftFrontPower * motorScaler);
            frontRightDrive.setPower(rightFrontPower * motorScaler);

            // Report on the state of the motor.
            int backLeftPos = backLeftDrive.getCurrentPosition();
            int backRightPos = backRightDrive.getCurrentPosition();
            int frontLeftPos = frontLeftDrive.getCurrentPosition();
            int frontRightPos = frontRightDrive.getCurrentPosition();
            telemetry.addData("mtr-lb", "%.1f (%.1f), pos=%d", leftBackPower * motorScaler, motorScaler, backLeftPos);
            telemetry.addData("mtr-rb", "%.1f (%.1f), pos=%d", rightBackPower * motorScaler, motorScaler, backRightPos);
            telemetry.addData("mtr-lf", "%.1f (%.1f), pos=%d", leftFrontPower * motorScaler, motorScaler, frontLeftPos);
            telemetry.addData("mtr-rf", "%.1f (%.1f), pos=%d", rightFrontPower * motorScaler, motorScaler, frontRightPos);
        }

        // sample slopper
        double sampleSlopperNewPosition = 0.5;
        if (gamepad1.y) {
            sampleSlopperNewPosition = 0.0;
        }
        sampleSlopper.setPosition(sampleSlopperNewPosition);




        if (doExtendoArm) {
            // Claw Servo
            double funkyClawNewPosition = 0.5;
            if (gamepad1.right_bumper) {
                funkyClawNewPosition = 1.0;
            } else if (gamepad1.left_bumper) {
                funkyClawNewPosition = 0.0;
            }
            funkyClaw.setPosition(funkyClawNewPosition);

            telemetry.addData("funcClaw", "actual=%.1f, desire=%.1f",
                    funkyClaw.getPosition(), funkyClawNewPosition);

            // Wrist servo
            int wristControlDirection = 0;
            if (gamepad1.left_trigger > 0.1) {
                wristControlDirection = 1;
            } else if (gamepad1.right_trigger > 0.1) {
                wristControlDirection = -1;
            }
            /*if (gamepad1.dpad_right) {
                wristControlDirection = 1;
            } else if (gamepad1.dpad_left) {
                wristControlDirection = -1;
            } */

            double wristCurrent = funkyWrist.getPosition();
            double clampedPosition = 0.5;
            if (wristControlDirection != 0) {
                //double wristStep = 0.002; // the bigger this is, the faster it will move
                double wristStep = 0.02; // the bigger this is, the faster it will move
                double newPosition = wristCurrent + (wristStep * wristControlDirection);
                clampedPosition = Math.max(0.5, Math.min(1.0, newPosition));
                funkyWrist.setPosition(clampedPosition);
            }
            telemetry.addData("funkWrst", "actual=%.1f, desired=%.1f, desired-dir=%d",
                    wristCurrent, clampedPosition, wristControlDirection);

            // Shoulder (DCmotor)
            double armMotorScaler = 1.0;
            switch (armThrottlerGear) {
            case GEAR_TWO:
                armMotorScaler = 0.5;
                break;
            case GEAR_ONE:
                armMotorScaler = 0.75;
                break;
            }
            double armPower = 0.0;
            if (gamepad1.dpad_up) {
                armPower = -1.0;
            } else if (gamepad1.dpad_down) {
                armPower = 1.0;
            }
            funkyShoulder.setPower(armPower * armMotorScaler);

            int funkyShoulderPos = funkyShoulder.getCurrentPosition();
            telemetry.addData("funkShdr", "%.1f (%.1f), pos=%d",
                    armPower * armMotorScaler, armMotorScaler, funkyShoulderPos);

            // Extendo
            double extendoMotorScaler = 1.0;
            double extendoPowerRaw = 0.0;
            int extendoDirection = 1;
            if (gamepad1.dpad_left) {
                extendoPowerRaw = 1.0;
                extendoDirection = -1;
            } else if (gamepad1.dpad_right) {
                extendoPowerRaw = 1.0;
            }
            double extendoPower = extendoPowerRaw * extendoMotorScaler * extendoDirection;
            funkyExtender.setPower(extendoPower);

            int funkyExtendoPos = funkyExtender.getCurrentPosition();
            telemetry.addData("funkXtnd", "%.1f (%.1f), pos=%d",
                    extendoPower, extendoPowerRaw, funkyExtendoPos);
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }
}
