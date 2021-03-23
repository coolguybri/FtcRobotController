package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="AutoDrop", group="AutoTest")
public class AutoDropWobbler extends AutoBase {

    @Override
    public void ratCrewGo() {

        closeGate();
        ratCrewWaitMillis(500);
        encoderDrive(65);
        turnRight(90);
        encoderDrive(24);
        openGate();
        ratCrewWaitMillis(500);
        encoderDrive(-24);
    }
}
