package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Close Two InDeep", group="InDeep")
public class RedCloseTwo extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(38);
        // plopThePurplePixel();
        turnRight(90);
        encoderDrive(48);
        turnRight(90);
        encoderDrive(27);
    }

}