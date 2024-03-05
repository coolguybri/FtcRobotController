package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="ARM ONE", group="DanceDemo")
public class FunkyShoulderTest extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        moveArmDown(1000);
        moveArmUp(500);
        moveArmDown(500);

        // test change.
    }
}
