package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Blue Turn Decode", group="AADecode")
public class AutoBlueTurn extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(12);
        turnLeft(45);
        ratCrewWaitSecs(2);
        encoderDrive(12);
    }

}