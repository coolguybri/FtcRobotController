package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Home Row Two", group="AAAInDeep")
public class RedHomeRowTwo extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(-4);
        ratCrewWaitMillis(500);
        turnLeft(90);
        ratCrewWaitMillis(500);
        encoderDrive(-45);
        plopThePurplePixel();
        encoderDrive(116);
    }

}