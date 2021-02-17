package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.concurrent.TimeUnit;

/**
 */
@Autonomous(name="Ring Seeker", group="AutoTest")
public class AutoRingSeeker extends AutoBase {

    @Override
    public void ratCrewGo() {
        ratCrewWaitMillis(500);
        RingConfig rc = ringIdentifier();
        printStatus();

        //TODO: debug only, remove 5 sec wait
        ratCrewWaitMillis(2000);
        telemetry.addData("RingSeek", "rc=%s", rc);
        telemetry.update();

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
            turnLeft(90);
            //encoderDrive(24);
        } else if (rc == RingConfig.SINGLE){
            turnRight(90);
            //encoderDrive(24);
        } else {
            turnRight(90);
            turnLeft(90);
        }

    }
}
