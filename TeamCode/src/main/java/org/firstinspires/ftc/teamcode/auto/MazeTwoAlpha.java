package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Maze Two PeterPiper", group="DanceDemo")
public class MazeTwoAlpha extends AutoDriveTest {

    @Override
    public void ratCrewGo() {

        /// begin
        encoderDrive(35); //  drives forward X inches

        turnLeft(45); // turns x degrees
        ratCrewWaitSecs(1);
        encoderDrive(57); //  drives forward X inches 75 TTOO MUCH
        ratCrewWaitSecs(1);
        //turnLeft(90);too much 90
        turnLeft(80);

        //encoderDrive(75); //  drives forward X inches
        encoderDrive(70); //  drives forward X inches



        turnLeft(90);
        encoderDrive(30);

        turnLeft(150);
        encoderDrive(125);
        turnRight(96);
        encoderDrive(75);
        turnRight(54);
        encoderDrive(75);
        /*
        turnRight(10);
        encoderDrive(92);
        turnRight(90);
        encoderDrive(85);
        turnRight(45);
        encoderDrive(45); */




        /*

        encoderDrive(29); //  drives forward X inches

        // turnLeft(90); // not far enough! crashed right pole
        //turnLeft(105); //tooo far! crashed into left pole!
        //turnLeft(97); //still too far, hit the pole
        turnLeft(94); // turns x degrees   <-------


        //encoderDrive(72); // did not drive far enough!
        encoderDrive(93); //  drives forward X inches;
        ratCrewWaitSecs(1);
        // turnRight(105); turned to much
        turnRight(94);

        // up til here works

        encoderDrive(96); //72 was too far 54 was too short
        turnRight(94);


        encoderDrive(71); // 59 was not far enough didn't cross the truss
        turnRight(58);
        encoderDrive(60);
        */



        // test change.
    }
}
