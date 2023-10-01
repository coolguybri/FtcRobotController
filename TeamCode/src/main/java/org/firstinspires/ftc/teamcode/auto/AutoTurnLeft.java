package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Rotate Left 90", group="AutoTest")
public class AutoTurnLeft extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        turnLeft(90);
    }
}
