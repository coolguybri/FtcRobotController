package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Rotate Right 90", group="AutoTest")
public class AutoTurnRight extends AutoDriveTest     {

    @Override
    public void ratCrewGo() {
        turnRight(90);
    }
}
