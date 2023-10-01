package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Straight 48", group="AutoTest")
public class AutoStraight48 extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(48);
        ratCrewWaitSecs(5);
    }
}
