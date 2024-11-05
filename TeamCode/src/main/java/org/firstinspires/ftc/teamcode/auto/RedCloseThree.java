package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Close Three InDeep", group="AAAInDeep")
public class RedCloseThree extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(-1);
        turnLeft(90);
        ratCrewWaitSecs(2);

        encoderDrive(-70);
        plopThePurplePixel();

        //turnRight(90);
        //encoderDrive(-48);
        //turnRight(90);
        //encoderDrive(-27);
    }

}