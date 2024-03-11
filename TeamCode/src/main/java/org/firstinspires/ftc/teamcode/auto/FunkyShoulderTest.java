package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="ARM ONE", group="DanceDemo")
public class FunkyShoulderTest extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        moveArmDown(2000);
        ratCrewWaitMillis(500);
        openFinger();
        ratCrewWaitMillis(500);
        closeFinger();


        // test change.
    }
}
