package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="ArmTest", group="AutoTest")
public class RedArmControl extends AutoDriveTest {


    //private int turnstomax = 4130;
    private int turnstomax = 4000;
    private int extendtomax = -2450;
    //private int extendtomax = -50;

    @Override
    public void ratCrewGo() {

        for (int i = 0 ; i < 2 ; i++) {
            moveArmCounter(turnstomax, 0.25f);
            ratCrewWaitMillis(1000);

            moveextendcounter(extendtomax, 0.20f);
            ratCrewWaitMillis(1000);
            moveextendcounter(0, 0.20f);
            ratCrewWaitMillis(1000);

            moveArmCounter(0, 0.25f);
            ratCrewWaitMillis(1000);
        }


        // plopThePurplePixel()

        //encoderDrive(-4);
        //ratCrewWaitMillis(500);
      //  turnLeft(90);
        //ratCrewWaitMillis(500);
        //encoderDrive(-22);
        //ratCrewWaitMillis(1000)
        //moveArmCounter(turnstomax, 0.25f);
        //ratCrewWaitMillis(1000);
        //moveextendcounter(extendtomax, 0.25f);
        //ratCrewWaitMillis(1000);
        //moveextendcounter(0, 0.25f);

        //ratCrewWaitMillis(1000);

       // moveArmCounter(0, 0.25f);
        //ratCrewWaitMillis(1000);
        //encoderDrive(96);
    }

}