package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Park Only Three", group="ZZOld")
public class RedParkOnlyThree extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(-1);
        turnLeft(90);
        encoderDrive(24);
    }

}