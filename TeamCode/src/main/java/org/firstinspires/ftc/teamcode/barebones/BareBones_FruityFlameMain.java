package org.firstinspires.ftc.teamcode.barebones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame... Go!", group="AAA")
public class BareBones_FruityFlameMain extends OpMode {

    // Instance Members.
    private boolean doMotors = true;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private double motorScaler = 1.0f;

    private boolean doGate = true;
    private boolean gateClosed = false;
    private boolean gateLastTrigger = false;
    private Servo gate;


    private boolean doArm = true;
    private DcMotor arm;
    private double angleHand;
    private double armScaler = 1.0f;
    private Servo finger;
    private boolean fingerClosed = false;
    private boolean fingerLastTrigger = false;



    // Called once, right after hitting the Init button.
    @Override
    public void init() {

        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            backLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftRear");
            backRightDrive = hardwareMap.get(DcMotor.class, "motorRightRear");
            frontLeftDrive = hardwareMap.get(DcMotor.class, "motorLeftFront");
            frontRightDrive = hardwareMap.get(DcMotor.class, "motorRightFront");
            backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            backRightDrive.setDirection(DcMotor.Direction.FORWARD);
            frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
            frontRightDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power.
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);


            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motorScaler = 1.0;
        }

        if (doGate) {
            // Initialize Motors, finding them through the hardware map.
            gate = hardwareMap.get(Servo.class, "gate");
            gateClosed = false;
            gateLastTrigger = false;

            // init the servo position
            clawRelease();
        }

        if (doArm) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            armScaler = 0.8;

            // init the servo position
            finger = hardwareMap.get(Servo.class, "finger");
            fingerClosed = false;
            fingerLastTrigger = false;
            finger.setPosition(0.0);
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized Gate, servo=%b", doGate);
    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        // Change the motorScalers.
        if (gamepad1.a) {
            motorScaler = 1.0;
        } else if (gamepad1.b) {
            motorScaler = 0.5;
        } else if (gamepad1.y) {
            armScaler = 0.4;
        } else if (gamepad1.x) {
            armScaler = 0.8;
        }

        if (doMotors) {
            double leftBackPower = 0.0;
            double rightBackPower = 0.0;
            double leftFrontPower = 0.0;
            double rightFrontPower = 0.0;

            // Look for digital controls
                //keep drive the same, turn--switch to right joystick
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                double strafe = gamepad1.left_stick_x;
                if(Math.abs(turn) > 0.1) {
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

        if (doGate) {
            // find the trigger for state change.
            double gateDetail = gamepad1.left_trigger;
            if (gateDetail != 0.0) {
                clawSet(gateDetail);
            } else {
                boolean thisTrigger = gamepad1.left_bumper;
                if (thisTrigger && !gateLastTrigger) {
                    this.gateClosed = !this.gateClosed;
                }
                gateLastTrigger = thisTrigger;

                if (this.gateClosed) {
                    clawPince();
                } else {
                    clawRelease();
                }
            }
        }

        if (doArm) {
            // controls the actual arm
            double armPower = 0.0;
            if (gamepad1.dpad_up) {
                armPower = 1.0;
            } else if (gamepad1.dpad_down) {
                armPower = -1.0;
            }

            arm.setPower(armPower * armScaler);
            telemetry.addData("arm", "%.1f (%.1f x %.1f)", armPower * armScaler, armPower, armScaler);

            // controls the finger with the right trigger
            double fingerDetail = gamepad1.right_trigger;
            if (Math.abs(fingerDetail) > 0.1) {
                finger.setPosition(1.0f - fingerDetail);
            } else {
                boolean thisTrigger = gamepad1.right_bumper;
                if (thisTrigger && !fingerLastTrigger) {
                    this.fingerClosed = !this.fingerClosed;
                }
                fingerLastTrigger = thisTrigger;

                if (this.fingerClosed) {
                    finger.setPosition(1.0);
                } else {
                    finger.setPosition(0.0);
                }


            }

            telemetry.addData("finger", "%.1f", finger.getPosition());
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }



    private void clawPince() {
        clawSet(0.8);
    }

    private void clawRelease() {
        clawSet(0.2);
    }

    private void clawSet(double newValue) {
        if (doGate) {
            if (newValue < 0.2)
                newValue = 0.2;
            angleHand = newValue;
            gate.setPosition(newValue);
        }
    }
}
