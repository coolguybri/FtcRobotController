package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Straight 96", group="AutoTest")
public class AutoStraight96 extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(96);
        ratCrewWaitSecs(5);
    }
}


