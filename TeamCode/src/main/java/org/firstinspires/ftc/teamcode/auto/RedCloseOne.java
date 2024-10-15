package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Close One InDeep", group="InDeep")
public class RedCloseOne extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(22);
    }

}