package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Blue Near SP", group="SP")
public class SimpleParkBlueNear extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(30);
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(38);
        plopThePurplePixel();
    }

}