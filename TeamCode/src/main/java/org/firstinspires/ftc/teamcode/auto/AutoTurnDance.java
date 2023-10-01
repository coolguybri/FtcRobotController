package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Rotate Dance 180", group="AutoTest")
public class AutoTurnDance extends AutoDriveTest {

    @Override
    public void ratCrewGo() {

        turnLeft(180);
        ratCrewWaitMillis(100);

        turnRight(180);
        ratCrewWaitMillis(100);
    }
}
