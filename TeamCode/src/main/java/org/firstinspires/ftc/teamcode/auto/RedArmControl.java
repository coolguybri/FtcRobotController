package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="RedArmControl", group="AAAInDeep")
public class RedArmControl extends AutoDriveTest {


    private int turnstomax = 3000;
    private int extendtomax = 50;

    @Override
    public void ratCrewGo() {

        moveArmCounter(turnstomax);
        ratCrewWaitMillis(500);
        moveextendcounter(extendtomax, 0.5f);
        ratCrewWaitMillis(500);
        moveextendcounter(-extendtomax, 0.5f);
        //ratCrewWaitMillis(500);
        //moveArmCounter(-turnstomax);
        ratCrewWaitMillis(1000);



        // plopThePurplePixel()


        //encoderDrive(-4);
        //ratCrewWaitMillis(500);
      //  turnLeft(90);
        //ratCrewWaitMillis(500);
        //encoderDrive(-22);
        //plopThePurplePixel();
        //encoderDrive(96);


    }

}