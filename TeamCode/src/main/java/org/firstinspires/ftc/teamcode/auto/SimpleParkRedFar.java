package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Far SP", group="SP")
public class SimpleParkRedFar extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(30);
        turnRight(90);
        ratCrewWaitSecs(2);
        encoderDrive(85);
        plopThePurplePixel();
    }

}