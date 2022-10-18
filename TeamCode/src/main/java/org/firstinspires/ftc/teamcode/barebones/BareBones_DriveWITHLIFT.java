package org.firstinspires.ftc.teamcode.barebones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
//@TeleOp(name="FINALDRIVE", group="AAA")
public class BareBones_DriveWITHLIFT extends OpMode {

    // Instance Members.
    private boolean doMotors = true;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private boolean useArm = true;
    private boolean useClaw = true;

    private boolean useShooter = true;
    private DcMotor sho;
    private DcMotor oter;

    private DcMotor armMotor;
    private Servo clawServo;

    private Servo gate;

    private double angleHand;


    // Called once, right after hitting the Init button.
    @Override
    public void init() {

        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");
            gate = hardwareMap.get(Servo.class, "Gate");
            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);
            gate.setPosition(1.0);

            // Set all motors to zero power.
            leftDrive.setPower(0);
            rightDrive.setPower(0);

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);

        if (useArm) {
            // Initialize Motors, finding them through the hardware map.
            armMotor = hardwareMap.get(DcMotor.class, "motorArm");
            armMotor.setDirection(DcMotor.Direction.FORWARD);

            // arm to Zero power
            armMotor.setPower(0);

            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (useClaw) {
            // Initialize Motors, finding them through the hardware map.
            clawServo = hardwareMap.get(Servo.class, "servoClaw");
        }

        if(useShooter) {
            sho = hardwareMap.get(DcMotor.class, "sho");
            oter = hardwareMap.get(DcMotor.class, "oter");
        }

            telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
            telemetry.addData("Yo", "Initialized Arm, motors=%b", useArm);
            telemetry.addData("Yo", "Initialized Claw, servo=%b", useClaw);
    }


    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        if (doMotors) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean shut = gamepad1.a;
            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("left", "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);

            if(shut) {
                gate.setPosition(0.0);
            } else {
                gate.setPosition(1.0);
            }
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());


        if (useArm) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            boolean armUp = gamepad1.dpad_up;
            boolean armDown = gamepad1.dpad_down;

            if (armUp) {
                armMotor.setPower(-0.5);
            }
            else if (armDown){
                armMotor.setPower(0.5);
            }
            else {
                armMotor.setPower(0);
            }
        }

        if (useClaw) {
            if (gamepad1.left_bumper) {
                clawPince();
            }
            else if (gamepad1.right_bumper) {
                clawRelease();
            }
        }

        if(useShooter) {
            float shooter = gamepad1.right_trigger;
            if(shooter > 0.0) {
                sho.setPower(1.0);
                oter.setPower(1.0);
            } else {
                sho.setPower(0.0);
                oter.setPower(0.0);
            }
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
}


    private void clawPince() {
        if (useClaw) {
            angleHand = 0.2;
            clawServo.setPosition(angleHand);
        }
    }

    private void clawRelease() {
        if (useClaw) {
            angleHand = 0.8;
            clawServo.setPosition(angleHand);
        }
    }
}
