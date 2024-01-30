package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoDriveTest;

/**
 */
@Autonomous(name="Blue Far SP", group="SP")
public class SimpleParkBlueFar extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(29);
        turnLeft(90);
        ratCrewWaitSecs(2);
        encoderDrive(87);
        plopThePurplePixel();
    }

}