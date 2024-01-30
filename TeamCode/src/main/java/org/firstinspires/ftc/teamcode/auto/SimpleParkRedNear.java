package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Near SP", group="SP")
public class SimpleParkRedNear extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(30);
        turnRight(90);
        ratCrewWaitSecs(2);
        encoderDrive(38);
        plopThePurplePixel();
    }

}