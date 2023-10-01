package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Knight Left", group="AutoTest")
public class AutoKnightLeft extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(55);
        turnLeft(90);
        encoderDrive(20);
    }
}
