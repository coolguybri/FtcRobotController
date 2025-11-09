package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Turn Decode", group="AADecode")
public class AutoRedTurn extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(12);
        turnRight(45);
        ratCrewWaitSecs(2);
        encoderDrive(12);
    }

}

