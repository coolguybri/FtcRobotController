package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="FullTilt 10", group="AutoTest")
public class AutoFullDrive10 extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        fullTiltForward(10);
        ratCrewWaitSecs(5);
    }
}


