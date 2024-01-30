package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Sasha Dance", group="DanceDemo")
public class DanceSasha extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(54);
        turnRight(180);
        ratCrewWaitSecs(1);
        turnLeft(270);
        encoderDrive(-18);
        encoderDrive(18);
        turnRight(360);
        encoderDrive(18);
        turnLeft(90);
        encoderDrive(18);
    }

}