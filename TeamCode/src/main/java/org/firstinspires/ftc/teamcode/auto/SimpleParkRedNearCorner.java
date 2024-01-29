package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Near SPC", group="SP")
public class SimpleParkRedNearCorner extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(6);
        turnRight(90);
        ratCrewWaitSecs(2);
        encoderDrive(38);
        plopThePurplePixel();
    }

}