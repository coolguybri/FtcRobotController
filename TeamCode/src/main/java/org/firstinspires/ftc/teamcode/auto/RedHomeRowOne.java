package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Red Home Row One", group="AAAInDeep")
public class RedHomeRowOne extends AutoDriveTest {

    @Override
    public void ratCrewGo() {

        // plopThePurplePixel()


        encoderDrive(-4);
        ratCrewWaitMillis(500);
        turnLeft(90);
        ratCrewWaitMillis(500);
        encoderDrive(-22);
        plopThePurplePixel();
        encoderDrive(96);
    }

}