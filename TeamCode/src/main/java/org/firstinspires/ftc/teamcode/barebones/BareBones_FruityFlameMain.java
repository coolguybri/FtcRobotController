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
    private boolean doGate = true;
    private boolean doArm = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor arm;

    private Servo gate;
    private Servo finger;

    private double angleHand;



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
        }

        if (doGate) {
            // Initialize Motors, finding them through the hardware map.
            gate = hardwareMap.get(Servo.class, "gate");
        }

        if(doArm) {
            arm = hardwareMap.get(DcMotor.class, "arm");
            finger = hardwareMap.get(Servo.class, "finger");
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized Gate, servo=%b", doGate);

    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        if (doMotors) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("left", "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);
        }

        if (doGate) {
            if (gamepad1.left_bumper) {
                clawPince();
            } else if (gamepad1.right_bumper) {
                clawRelease(); 
            }
        }

        if(doArm) {
            //controls the actual arm
            if (gamepad1.dpad_up) {
                arm.setPower(1.0);
            } else if (gamepad1.dpad_down) {
                arm.setPower(-1.0);
            } else {
                arm.setPower(0);
            }

            //controls the finger with the right trigger
            if (gamepad1.right_trigger > 0) {
                finger.setPosition(1.0);
            } else if (gamepad1.right_trigger <= 0) {
                finger.setPosition(0.0);
            }
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }



    private void clawPince() {
        if (doGate) {
            angleHand = 0.2;
            gate.setPosition(angleHand);
        }
    }

    private void clawRelease() {
        if (doGate) {
            angleHand = 0.8;
            gate.setPosition(angleHand);
        }
    }
}
