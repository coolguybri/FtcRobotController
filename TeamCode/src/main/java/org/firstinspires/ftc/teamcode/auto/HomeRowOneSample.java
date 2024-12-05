package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Home Row One Sample", group="AAAInDeep")
public class HomeRowOneSample extends AutoDriveTest {

    @Override
    public void ratCrewGo() {

        // plopThePurplePixel()

        turnRight(90);
        encoderDrive(-24);
        turnLeft(90);
        moveArmCounter(3482, 1);
        encoderDrive(-24);
        moveArmCounter(1, 1);
        encoderDrive(24);


        //encoderDrive(-4);
        //ratCrewWaitMillis(500);
        //turnLeft(90);
        //ratCrewWaitMillis(500);
        //encoderDrive(-22);
        //plopThePurplePixel();
        //encoderDrive(96);
    }

}