package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Rotate Right 180", group="AutoTest")
public class AutoTurnAroundRight extends AutoBase {

    @Override
    public void ratCrewGo() {
        turnRight(180);
    }
}
