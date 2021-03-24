package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Final Mode", group="A")
public class AutoFinalMode extends AutoBase {

    @Override
    public void ratCrewGo() {
        closeGate();
        ratCrewWaitMillis(500);
        RingConfig rc = ringIdentifier();
        printStatus();

        ratCrewWaitMillis(2000);
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


        if (rc == RingConfig.QUAD){
            thirdBox();
            openGate();

        } else if (rc == RingConfig.SINGLE){
            secondBox();
            openGate();
        } else {
            firstBox();
            openGate();
        }

    }
}
