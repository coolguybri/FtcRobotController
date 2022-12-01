package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Rotate Left 180", group="AutoTest")
public class AutoTurnAroundLeft extends AutoBase {

    @Override
    public void ratCrewGo() {
        turnLeft(180);
    }
}
