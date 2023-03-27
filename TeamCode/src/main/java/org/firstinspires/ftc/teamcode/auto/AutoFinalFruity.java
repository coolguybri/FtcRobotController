package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Fruity Final Mode", group="A")
public class AutoFinalFruity extends AutoBase {

    @Override
    public void ratCrewGo() {
        openGate();
        ratCrewWaitMillis(500);
        SignalConfig rc = SignalIdentifier();
        printStatus();

        ratCrewWaitMillis(500);
        telemetry.addData("RingSeek", "rc=%s", rc);
        telemetry.update();

        telemetry.addData("RingConfig", "%s", rc);

        //TODO: Uncomment if needed
        /*
        long currentTime = System.nanoTime();
        long loopCrasher = TimeUnit.SECONDS.toNanos(4L);
        int forMove = 0;

        while (ringIdentifier() == RingConfig.EMPTY){
            encoderDrive(24);
            forMove = +24;
            sleep(5);
            ringIdentifier();

            if (currentTime > msStuckDetectLoop - 10) {
                break;
            }
        }
        encoderDrive(-forMove); */


        if (rc == SignalConfig.ONE){

            moveArmUp(1000);

            encoderDrive(6);
            turnLeft(90);
            encoderDrive(23);
            turnRight(90);
            encoderDrive(37);


        } else if (rc == SignalConfig.TWO){

            arm.setPower(-0.7);
            ratCrewWaitMillis(1000);
            arm.setPower(0);

            encoderDrive(43);


        } else if (rc == SignalConfig.THREE){

            moveArmUp(1000);

            encoderDrive(6);
            turnRight(90);
            encoderDrive(23);
            turnLeft(90);
            encoderDrive(37);


        } else {
            encoderDrive(43);
        }

        //tighten up the robot!
        closeGate();
        closeFinger();
        moveArmDown(500);


    }
}
