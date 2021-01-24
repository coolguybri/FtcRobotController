package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Turn Around", group="AutoTest")
public class AutoTurnAround extends AutoBase {

    @Override
    public void ratCrewGo() {
        turnLeft(180);
    }
}
