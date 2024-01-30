package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Easton Dance", group="DanceDemo")
public class DanceEaston extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(29);
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(87);
        plopThePurplePixel();
    }

}