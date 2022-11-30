package org.firstinspires.ftc.teamcode.barebones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame... Go!", group="AAA")
public class BareBones_FruityFlameMain extends OpMode {

    // Instance Members.
    private boolean doMotors = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
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
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power.
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            armScaler = 0.6;

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
            armScaler = 0.3;
        } else if (gamepad1.x) {
            armScaler = 0.6;
        }

        if (doMotors) {
            double leftPower = 0.0;
            double rightPower = 0.0;

            // Look for digital controls
            if (gamepad1.dpad_up) {
                leftPower = rightPower = 1.0;
            } else if (gamepad1.dpad_down) {
                leftPower = rightPower = -1.0;
            } else if (gamepad1.dpad_left) {
                leftPower = -1.0;
                rightPower = 1.0;
            } else if (gamepad1.dpad_right) {
                leftPower = 1.0;
                rightPower = -1.0;
            } else {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.left_stick_x;
                leftPower = Range.clip(drive + turn, -1.0, 1.0);
                rightPower = Range.clip(drive - turn, -1.0, 1.0);
            }

            // Send calculated power to wheels
            leftDrive.setPower(leftPower * motorScaler);
            rightDrive.setPower(rightPower * motorScaler);
            telemetry.addData("left", "%.1f (%.1f x %.1f)", leftPower * motorScaler, leftPower, motorScaler);
            telemetry.addData("right", "%.1f (%.1f x %.1f)", rightPower * motorScaler, rightPower, motorScaler);
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
            double armControl = gamepad1.right_stick_y;
            double armPower = Range.clip(armControl, -1.0, 1.0);
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
