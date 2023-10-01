package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Meditate", group="AutoTest")
public class AutoMeditate extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        ratCrewWaitMillis(100000);
    }

}
