package org.firstinspires.ftc.teamcode.barebones;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="Fruity Flame BattleBot!", group="AAA")
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

    private boolean vigourousAttack = false;
    private int vigourousState = 0;

    private boolean doTail = true;
    private Servo tail; // The servo motor device
    private boolean tailAttackPressed = false; // If the button pressed?
    private int tailAttackCounter = 0;
    private boolean tailDirection = true;

    private boolean doOnePuncher = true;
    private DcMotor puncher;

    private boolean doDriller = true;
    private DcMotor driller;


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

        // TODO: this gate is not used! Delete it!
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
            armScaler = 1.0;

            // init the servo position
            finger = hardwareMap.get(Servo.class, "finger");
            fingerClosed = false;
            fingerLastTrigger = false;
            finger.setPosition(0.0);
        }

        if (doTail) {
            // Initialize Motors, finding them through the hardware map.
            tail = hardwareMap.get(Servo.class, "tail");
            tail.setPosition(0.0);
        }

        /* One Puncher Weapon! */
        boolean errOnePuncher = false;
        if (doOnePuncher) {
            try {
                puncher = hardwareMap.get(DcMotor.class, "puncher");
                puncher.setDirection(DcMotor.Direction.REVERSE);
                puncher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                puncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                puncher.setPower(0);
            } catch (Exception e) {
                doOnePuncher = false;
                errOnePuncher = true;
            }
        }

        /* One Puncher Weapon! */
        boolean errDriller = false;
        if (doDriller) {
            try {
                driller = hardwareMap.get(DcMotor.class, "driller");
                driller.setDirection(DcMotor.Direction.REVERSE);
                driller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                driller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                driller.setPower(0);
            } catch (Exception e) {
                doDriller = false;
                errDriller = true;
            }
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);
        telemetry.addData("Yo", "Initialized Gate, servo=%b", doGate);
        telemetry.addData("Yo", "Initialized OnePunchMan, do=%s, err=%s", doOnePuncher, errOnePuncher);
        telemetry.addData("Yo", "Initialized Driller, do=%s, err=%s", doDriller, errDriller);
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

        // Special Vigorous Atack Mode
        if (gamepad1.left_bumper) {
            if (!vigourousAttack) {
                vigourousAttack = true;
                vigourousState = 0;
            }

            // increment state.
            vigourousState++;

            // Compute direction change time.
            double shakePower = 1.0;
            boolean direction = true;
            int directionDuration = 15;
            int directionShift = (vigourousState % (directionDuration * 2));
            if (directionShift > directionDuration) {
                direction = !direction;
            }

            if (direction) {
              nudgeLeft(shakePower);
            } else {
              nudgeRight(shakePower);
            }
            try {
                sleep(20);
            } catch (Exception e) {
            }
        } else {
            vigourousAttack = false;
            vigourousState = 0;
        }


        // Detect special Tail Attack Mode
        /*if (gamepad1.right_bumper) {
            // Detect if they just pressed the button down for the first time (as opposed to holding it down for a while.
            // Reset the counter, and the iniitla direction.
            if (!tailAttackPressed) {
                tailAttackPressed = true;
                tailAttackCounter = 0;
                tailDirection = true;
            }

            // increment the counter - the number of program loops we have had the button down.
            tailAttackCounter++;

            // Every X loops, switch directions. Detect this by swtiching evertime the counter hits a number divisible by 15.
            int directionShiftTrigger = (tailAttackCounter % 250);
            if (directionShiftTrigger == 0) {
                tailDirection = !tailDirection;
            }

            // Set the servo to the farthest position for whatever direction it is supposed to be in right now.
            if (tailDirection) {
                tail.setPosition((0.0));
            } else {
                tail.setPosition((1.0));
            }

        } else {
            tailAttackPressed = false;
            tailAttackCounter = 0;
        } */

        if (doMotors) {
            double leftBackPower = 0.0;
            double rightBackPower = 0.0;
            double leftFrontPower = 0.0;
            double rightFrontPower = 0.0;

            // Look for digital controls
                //keep drive the same, turn--switch to right joystic
                double drive = -gamepad1.left_stick_y;
                if (Math.abs(drive) < 0.1) {
                    drive = -gamepad1.right_stick_y;
                }
                double turn = gamepad1.right_stick_x;
                double strafe = -gamepad1.left_stick_x;
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

       /* if (doGate) {
            // find the trigger for state change.
            double gateDetail = gamepad1.left_trigger;
            if (gateDetail != 0.0) {
                clawSet(gateDetail);
            } /* else {
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
        } */

        if (doDriller) {
            double drive = gamepad1.left_trigger;
            driller.setPower(drive);
            telemetry.addData("driller", "%.1f", drive);
        }

        if (doArm) {
            // controls the actual arm
            double armPower = 0.0;
            if (gamepad1.dpad_up) {
                armPower = -1.0;
            } else if (gamepad1.dpad_down) {
                armPower = 1.0;
            }

            arm.setPower(armPower * armScaler);
            telemetry.addData("arm", "%.1f (%.1f x %.1f)", armPower * armScaler, armPower, armScaler);

            // controls the finger with the right trigger
            double fingerDetail = gamepad1.right_trigger;
            if (Math.abs(fingerDetail) > 0.1) {
                finger.setPosition(1.0f - fingerDetail);
            }

            telemetry.addData("finger", "%.1f", finger.getPosition());
        }

        if (doOnePuncher) {

            // Status update.
            int currPos = puncher.getCurrentPosition();
            telemetry.addData("puncher", "curr=%d", currPos);

            if (gamepad1.right_bumper) {
                double punchPower = 1.0;
                int distance = 900;
                telemetry.addData("puncher2", "curr=%d", currPos);

                ElapsedTime motorOnTime;
                int startPos = puncher.getCurrentPosition();
                int endPos = (int) startPos + distance;

                // Punch out
                puncher.setTargetPosition(endPos);
                puncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorOnTime = new ElapsedTime();
                puncher.setPower(punchPower);
                while (puncher.isBusy() && motorOnTime.seconds() < 5) {
                    currPos = puncher.getCurrentPosition();
                    telemetry.addData("puncher-move", "start=%d, end=%d, curr=%d, dist=%d, busy=%b)",
                            startPos, endPos, currPos, distance, puncher.isBusy());
                    telemetry.update();
                }
                puncher.setPower(0);

                // Retract post-punch
                puncher.setTargetPosition(startPos);
                puncher.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorOnTime = new ElapsedTime();
                puncher.setPower(-punchPower);
                while (puncher.isBusy() && motorOnTime.seconds() < 5) {
                    currPos = puncher.getCurrentPosition();
                    telemetry.addData("puncher-move2", "start=%d, end=%d, curr=%d, dist=%d, busy=%b)",
                            startPos, endPos, currPos, distance, puncher.isBusy());
                    telemetry.update();
                }
                puncher.setPower(0);
            }

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
}
