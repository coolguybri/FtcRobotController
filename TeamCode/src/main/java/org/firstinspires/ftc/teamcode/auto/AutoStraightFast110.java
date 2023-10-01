package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Straight Fast 110", group="AutoTest")
public class AutoStraightFast110 extends AutoDriveTest {

    private int distance = 110;

    @Override
    public void ratCrewGo() {
        encoderDrive(distance, distance, 1.0);
        ratCrewWaitSecs(5);
    }
}


