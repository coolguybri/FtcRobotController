package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame: Decode!", group="AAA")
public class Teleop_Decode extends OpMode {

    // Throttler
    private enum ThrottlerGear {
        GEAR_TWO,
        GEAR_ONE,
    };

    // 4-wheel drive
    private boolean doMotors = true;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private boolean throttlerButtonLast = false;
    private ThrottlerGear throttlerGear = ThrottlerGear.GEAR_TWO;

    // FunkyArm
    private boolean doExtendoArm = true;
    private Servo funkyWrist;
    private DcMotor funkyShoulder;
    private DcMotor funkyShoulder2;
    private boolean funkyOn;
    private boolean armButtonLast;
    private boolean armThrottlerButtonLast = false;
    private ThrottlerGear armThrottlerGear = ThrottlerGear.GEAR_TWO;

    private boolean doGate = false;
    private Servo gate;

    // Called once, right after hitting the Init button.
    @Override
    public void init() {

        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            frontLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftFront");
            frontRightDrive = hardwareMap.get(DcMotor.class, "motorRightFront");
            backLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftRear");
            backRightDrive = hardwareMap.get(DcMotor.class, "motorRightRear");

            // HACK: this front right gear box was installed backward.
            //frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
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

        boolean errFunkyArm = false;
        if (doExtendoArm) {
            try {
                // wrist
                funkyWrist = hardwareMap.get(Servo.class, "funkyWrist");
                funkyWrist.setPosition(0.5f);

                // shoulder
                funkyShoulder = hardwareMap.get(DcMotor.class, "funkyShoulder");
                funkyShoulder.setDirection(DcMotor.Direction.FORWARD);
                funkyShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyShoulder.setPower(0);

                funkyShoulder2 = hardwareMap.get(DcMotor.class, "funkyShoulder2");
                funkyShoulder2.setDirection(DcMotor.Direction.REVERSE);
                funkyShoulder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyShoulder2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyShoulder2.setPower(0);

                funkyOn = false;
                armButtonLast = false;
            } catch (Exception e) {
                doExtendoArm = false;
                errFunkyArm = true;
                telemetry.addData("Error", "ExtendoArm: %s", e.getMessage());
            }
        }

        boolean errGate = false;
        if (doGate) {
            try {
                // wrist
                funkyWrist = hardwareMap.get(Servo.class, "gate");
                //funkyWrist.setPosition(0.5f);
            } catch (Exception e) {
                doGate = false;
                errGate = true;
                telemetry.addData("Error", "Gate: %s", e.getMessage());
            }
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized ExtendoArm, do=%s, err=%s", doExtendoArm, errFunkyArm);
        telemetry.addData("Yo", "Initialized Gate, do=%s, err=%s", doGate, errGate);
    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        if (doMotors) {

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

            double leftBackPower = 0.0;
            double rightBackPower = 0.0;
            double leftFrontPower = 0.0;
            double rightFrontPower = 0.0;

            double motorScaler = 1.0;
            switch (throttlerGear) {
            case GEAR_TWO:
                motorScaler = 1.0;
                break;
            case GEAR_ONE:
                motorScaler = 0.75;
                break;
            }

            double drive = -gamepad1.left_stick_y;
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

        if (doExtendoArm) {

            // Wrist servo
            int wristControlDirection = 0;
            if (gamepad1.left_trigger > 0.1) {
                wristControlDirection = 1;
            } else if (gamepad1.right_trigger > 0.1) {
                wristControlDirection = -1;
            }

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


            // shoulder throttler
            armThrottlerGear = ThrottlerGear.GEAR_ONE;
            boolean armThrottlerButtonNow = gamepad1.right_bumper;
            if (armThrottlerButtonNow) {
                armThrottlerGear = ThrottlerGear.GEAR_TWO;
            }

            // Shoulder (DCmotor)
            double armMotorScaler = 1.0;
            switch (armThrottlerGear) {
            case GEAR_TWO:
                armMotorScaler = 0.9;
                break;
            case GEAR_ONE:
                armMotorScaler = 0.6;
                break;
            }

            // Arm throttler: Left Bumper
            boolean armButtonNow = gamepad1.a;
            if (!armButtonLast && armButtonNow) {
                funkyOn = !funkyOn;
            }
            armButtonLast = armButtonNow;

            double armPower = 0.0;
            if (funkyOn) {
                armPower = 1.0;
            }
            funkyShoulder.setPower(armPower * armMotorScaler);
            funkyShoulder2.setPower(armPower * armMotorScaler);

            telemetry.addData("funkShdr", "%.1f (%.1f)",
                    armPower * armMotorScaler, armMotorScaler);
        }

        if (doGate) {

            // Wrist servo
            int gateControlDirection = 0;
            if (gamepad1.left_bumper) {
                gateControlDirection = 1;
            } else if (gamepad1.right_bumper) {
                gateControlDirection = -1;
            }

            double gateCurrent = gate.getPosition();
            double clampedPosition = 0.5;
            if (gateControlDirection != 0) {
                //double wristStep = 0.002; // the bigger this is, the faster it will move
                double wristStep = 0.02; // the bigger this is, the faster it will move
                double newPosition = gateCurrent + (wristStep * gateControlDirection);
                clampedPosition = Math.max(0.5, Math.min(1.0, newPosition));
                gate.setPosition(clampedPosition);
            }
            telemetry.addData("gate", "actual=%.1f, desired=%.1f, desired-dir=%d",
                    gateCurrent, clampedPosition, gateControlDirection);
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }
}
