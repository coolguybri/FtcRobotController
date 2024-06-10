package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="ARM ONE", group="DanceDemo")
public class    FunkyShoulderTest extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        //moveArmDown(2000);
        encoderDrive(20);
        wristSet(0.55F);
        ratCrewWaitMillis(1000);
        moveArmCounter(5300);



        //moveArmCounter(1000);
        openFinger();
        ratCrewWaitMillis(1000);
        closeFinger();
        ratCrewWaitMillis(1500);
        moveArmCounter(-1000);
        ratCrewWaitMillis(2000);

        encoderDrive(36);
        ratCrewWaitMillis(2000);
        turnRight(80);
        ratCrewWaitMillis(2000);
        encoderDrive(67);
        ratCrewWaitMillis(2000);
        turnRight(80);

        ratCrewWaitMillis(2000);
        encoderDrive(18);
        ratCrewWaitMillis(2000);
        turnLeft(80);
        ratCrewWaitMillis(2000);
        encoderDrive(10);
        ratCrewWaitMillis(2000);
        moveArmCounter(-1000);
        openFinger();
        // test change.
    }
}
