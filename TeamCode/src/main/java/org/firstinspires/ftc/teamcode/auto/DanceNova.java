package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Nova Dance", group="DanceDemo")
public class DanceNova extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(54);
        turnLeft(360);
        ratCrewWaitSecs(2);
        encoderDrive(-18);
        turnLeft(180);
        encoderDrive(18);
        encoderDrive(-18);
        turnLeft(180);
    }
}


