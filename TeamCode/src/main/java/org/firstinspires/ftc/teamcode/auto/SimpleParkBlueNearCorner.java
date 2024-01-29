package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Blue Near SPC", group="SP")
public class SimpleParkBlueNearCorner extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(6);
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(38);
        plopThePurplePixel();
    }

}