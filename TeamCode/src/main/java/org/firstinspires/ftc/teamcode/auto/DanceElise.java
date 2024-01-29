package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Elise Dance", group="DanceDemo")
public class DanceElise extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(18); //  drives forward X inches
        turnLeft(360); // turns x degrees
        encoderDrive(18); //  drives forward X inches
    }
}
