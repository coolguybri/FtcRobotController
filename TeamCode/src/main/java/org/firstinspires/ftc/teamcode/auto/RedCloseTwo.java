package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Close Two InDeep", group="AAAInDeep")
public class RedCloseTwo extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(-1);
        turnLeft(90);
        ratCrewWaitSecs(2);

        encoderDrive(-47);
        plopThePurplePixel();

        //turnRight(90);
        //encoderDrive(-47);
        //turnRight(90);
        //encoderDrive(-27);
    }
}