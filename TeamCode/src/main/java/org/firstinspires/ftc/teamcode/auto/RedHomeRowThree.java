package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Home Row Three", group="AAAInDeep")
public class RedHomeRowThree extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(-4);
        ratCrewWaitMillis(500);
        turnLeft(90);
        ratCrewWaitMillis(500);
        encoderDrive(-72);
        plopThePurplePixel();
        encoderDrive(96);
    }

}