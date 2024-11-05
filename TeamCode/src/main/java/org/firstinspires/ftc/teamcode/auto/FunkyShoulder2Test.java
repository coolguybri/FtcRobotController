package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="ARM Goofy", group="ZZ DanceDemo")
public class    FunkyShoulder2Test extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        //moveArmDown(2000);

        // attempt1
        moveArmCounter(2500);
        wristSet(0.5f);
        openFinger();
        ratCrewWaitMillis(1000);

       // moveArmCounter(3000);
        moveArmCounter(2150);


        ratCrewWaitMillis(500);
       // openFinger();
        ratCrewWaitMillis(500);
        closeFinger();
        ratCrewWaitMillis(1000);

        // attempt1
        moveArmCounter(-450);
        wristSet(0.6f);
        moveArmCounter(-450);
        //moveArmCounter(1000);



        // New code!
       // ratCrewWaitMillis(2000);
        //wristSet(0.5f);
      //  ratCrewWaitMillis(2000);
       // wristSet(1.0f);

        //moveArmCounter(-1000);


        // test change.
    }
}
