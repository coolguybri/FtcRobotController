package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame CenterStage!", group="OldTeleop")
public class Teleop_CenterStage extends OpMode {

    // Throttler
    private enum ThrottlerGear {
        GEAR_TWO,
        GEAR_ONE,
    };
    private boolean throttlerButtonLast = false;
    private ThrottlerGear throttlerGear = ThrottlerGear.GEAR_ONE;
    private boolean armThrottlerButtonLast = false;
    private ThrottlerGear armThrottlerGear = ThrottlerGear.GEAR_ONE;

    // Wheels
    private boolean doMotors = true;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;

    // FunkyArm
    private boolean doFunkyArm = true;
    private Servo funkyClaw;
    private boolean funkyClawLock = false;
    private boolean funkyClawButtonLast = false;
    private Servo funkyWrist;
    private DcMotor funkyShoulder;
    private DcMotor funkyShoulder2;
    private int funkyShoulderStartPos = 0;
    private int funkyShoulderEndPos = 0;

    // Drone Launcher
    private boolean doDroneLauncher = true;
    private Servo launcherLatch;

    // Piper's Purple Pixel Plopper
    private boolean doPurplePixelPlopper = true;
    private Servo purplePixelPlopper;

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
            frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
            backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
            backRightDrive.setDirection(DcMotor.Direction.REVERSE);

            // Set all motors to zero power.
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Set all motors to run without encoders; manual control.
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        boolean errFunkyArm = false;
        if (doFunkyArm) {
            try {
                // claw
                funkyClawButtonLast = false;
                funkyClawLock = true;
                funkyClaw = hardwareMap.get(Servo.class, "funkyClaw");
                funkyClaw.setPosition(0.0); // start clamped

                // wrist
                funkyWrist = hardwareMap.get(Servo.class, "funkyWrist");
                //funkyWrist.setPosition(0.0);

                 // shoulder
                funkyShoulder = hardwareMap.get(DcMotor.class, "funkyShoulder");
                funkyShoulder.setDirection(DcMotor.Direction.FORWARD);
                funkyShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyShoulder.setPower(0);

                funkyShoulder2 = hardwareMap.get(DcMotor.class, "funkyShoulder2");
                funkyShoulder2.setDirection(DcMotor.Direction.REVERSE);
                funkyShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                funkyShoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                funkyShoulder.setPower(0);

                funkyShoulderStartPos = funkyShoulder.getCurrentPosition();
                funkyShoulderEndPos = funkyShoulderStartPos + 200;
            } catch (Exception e) {
                doFunkyArm = false;
                errFunkyArm = true;
                telemetry.addData("Error", "FunkyArm: %s", e.getMessage());
            }
        }

        boolean errDroneLauncher = false;
        if (doDroneLauncher) {
            try {
                launcherLatch = hardwareMap.get(Servo.class, "launcherLatch");
                launcherLatch.setPosition(0.5);
            } catch (Exception e) {
                doDroneLauncher = false;
                errDroneLauncher = true;
                telemetry.addData("Error", "DroneLauncher: %s", e.getMessage());
            }
        }

        boolean errPurplePixelPlopper = false;
        if (doPurplePixelPlopper) {
            try {
                purplePixelPlopper = hardwareMap.get(Servo.class, "pixelPlopper");
                purplePixelPlopper.setPosition(0.0);
            } catch (Exception e) {
                doPurplePixelPlopper = false;
                errPurplePixelPlopper = true;
                telemetry.addData("Error", "PurplePixelPlopper: %s", e.getMessage());
            }
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized FunkyArm, do=%s, err=%s", doFunkyArm, errFunkyArm);
        telemetry.addData("Yo", "Initialized Launcher, do=%s, err=%s", doDroneLauncher, errDroneLauncher);
        telemetry.addData("Yo", "Initialized PPP, do=%s, err=%s", doPurplePixelPlopper, errPurplePixelPlopper);
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
        boolean armThrottlerButtonNow = gamepad1.left_bumper;
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
        armThrottlerButtonLast = armThrottlerButtonNow;

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
                leftBackPower = Range.clip(drive + turn, -1.0, 1.0);
                leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
                rightBackPower = Range.clip(drive - turn, -1.0, 1.0);
                rightFrontPower = Range.clip(drive - turn, -1.0, 1.0);
            } else {
                leftBackPower = Range.clip(drive - strafe, -1.0, 1.0);
                leftFrontPower = Range.clip(drive + strafe, -1.0, 1.0);
                rightBackPower = Range.clip(drive + strafe, -1.0, 1.0);
                rightFrontPower = Range.clip(drive - strafe, -1.0, 1.0);
            }

            // Send calculated power to wheels
            backLeftDrive.setPower(leftBackPower * motorScaler);
            backRightDrive.setPower(rightBackPower * motorScaler);
            frontLeftDrive.setPower(leftFrontPower * motorScaler);
            frontRightDrive.setPower(rightFrontPower * motorScaler);
            telemetry.addData("left", "%.1f (%.1f x %.1f)", leftBackPower * motorScaler, leftBackPower, motorScaler);
            telemetry.addData("right", "%.1f (%.1f x %.1f)", rightBackPower * motorScaler, rightBackPower, motorScaler);
            telemetry.addData("left", "%.1f (%.1f x %.1f)", leftFrontPower * motorScaler, leftFrontPower, motorScaler);
            telemetry.addData("right", "%.1f (%.1f x %.1f)", rightFrontPower * motorScaler, rightFrontPower, motorScaler);
        }

        if (doFunkyArm) {
            // Claw Servo
            boolean funkyClawButtonNow = gamepad1.right_bumper;
            if (!funkyClawButtonLast && funkyClawButtonNow) {
                funkyClawLock = !funkyClawLock;
            }
            funkyClawButtonLast = funkyClawButtonNow;

            double funkyClawNewPosition = 1.0;
            if (funkyClawLock) {
                funkyClawNewPosition = 0.0;
            }
            funkyClaw.setPosition(funkyClawNewPosition);
            telemetry.addData("funcClaw", "actual=%.1f, desire=%.1f", funkyClaw.getPosition(), funkyClawNewPosition);

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
            double clampedPosition = 0.0;
            if (wristControlDirection != 0) {
                double wristStep = 0.002; // the bigger this is, the faster it will move
                double newPosition = wristCurrent + (wristStep * wristControlDirection);
                clampedPosition = Math.max(0.0, Math.min(1.0, newPosition));
                funkyWrist.setPosition(clampedPosition);
            }
            telemetry.addData("funkWrst", "actual=%.1f, desired=%.1f, desired-dir=%d", wristCurrent, clampedPosition, wristControlDirection);

            // Shoulder (DCmotor)
            double armMotorScaler = 1.0;
            switch (armThrottlerGear) {
            case GEAR_TWO:
                armMotorScaler = 1.0;
                break;
            case GEAR_ONE:
                armMotorScaler = 0.5;
                break;
            }
            double armPower = 0.0;
            if (gamepad1.dpad_up) {
                armPower = -1.0;
            } else if (gamepad1.dpad_down) {
                armPower = 1.0;
            }
            funkyShoulder.setPower(armPower * armMotorScaler);
            funkyShoulder2.setPower(armPower * armMotorScaler);
            telemetry.addData("funkShdr", "%.1f (%.1f x %.1f), boost=%b", armPower * armMotorScaler, armPower, armMotorScaler, gamepad1.b);
        }

        if (doDroneLauncher) {
            if (gamepad1.a) {
                launcherLatch.setPosition(1.0);
            } else {
                launcherLatch.setPosition(0.5);
            }
        }

        if (doPurplePixelPlopper) {
           if (gamepad1.y) {
                purplePixelPlopper.setPosition(1.0);
            } else {
                 purplePixelPlopper.setPosition(0.0);
            }
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }
}
