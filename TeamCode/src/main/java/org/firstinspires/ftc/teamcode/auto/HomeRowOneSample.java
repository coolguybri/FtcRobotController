package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="One Low Bucket", group="AAAInDeep")
public class HomeRowOneSample extends AutoDriveTest {

    @Override
    public void ratCrewGo() {

        // plopThePurplePixel()

        // TEST CODE to show how ARM can move
        //encoderDrive(-24);
        //moveArmCounter(3000, 0.5f);
        /*ratCrewWaitMillis(2000);
        raiseWrist();
        closeFinger();
        encoderDrive(5);
        ratCrewWaitMillis(2000);
        resetFinger();
        lowerWrist();
        ratCrewWaitMillis(2000);
        moveArmCounter(1000, 0.5f);
        ratCrewWaitMillis(2000);
        closeFinger();
        ratCrewWaitMillis(2000);
        resetFinger();
        /*moveExtendCounter(-300, 0.75f);
        openFinger();
        ratCrewWaitMillis(2000);
        resetFinger();
        moveExtendCounter(50, 0.75f);
        ratCrewWaitMillis(500);
        //moveArmCounter(400, 0.5f); */

        //ratCrewWaitMillis(30000);

        //turnRight(90);
        //encoderDrive(-24);
        //turnLeft(90);
        //closeFinger();
       //moveArmCounter(1500, 1.0f);
       //wristSet(0.5f);
        //encoderDrive(-24);
        //moveArmCounter(20, 0.75f);
        //openFinger();
        //encoderDrive(20);


        //encoderDrive(-4);
        //ratCrewWaitMillis(500);
        //turnLeft(90);
        //ratCrewWaitMillis(500);
        //encoderDrive(-22);
        //plopThePurplePixel();
        //encoderDrive(96);

        encoderDrive(-5);
        turnLeft(90);
        ratCrewWaitSecs(2);
        moveArmCounter(2500, 0.5f);
        raiseWrist();
        encoderDrive(-22);
        turnLeft(45);
        moveArmCounter(2100, 0.5f);
        openFinger();
        ratCrewWaitSecs(4);

    }

}