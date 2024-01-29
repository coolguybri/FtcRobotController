package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Far SPC", group="SP")
public class SimpleParkRedFarCorner extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(6);
        turnRight(90);
        ratCrewWaitSecs(2);
        encoderDrive(84);
        plopThePurplePixel();
    }

}