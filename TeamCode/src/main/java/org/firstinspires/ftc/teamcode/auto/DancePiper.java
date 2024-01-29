package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Piper Dance", group="DanceDemo")
public class DancePiper extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(36);
        turnLeft(90);
        encoderDrive(36);
        turnLeft(180);
        encoderDrive(54);
        turnLeft(360);
        encoderDrive(-36);
        turnRight(360);
        turnLeft(360);
        plopThePurplePixel();
    }

}