package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Blue Far SPC", group="SP")
public class SimpleParkBlueFarCorner extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(5);
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(87);
        plopThePurplePixel();
    }
}