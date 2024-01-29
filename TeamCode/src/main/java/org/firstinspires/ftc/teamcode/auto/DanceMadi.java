package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Maddy Dance", group="DanceDemo")
public class DanceMadi extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(20); // forward
        turnRight(135);
        encoderDrive(10);
        turnLeft(90);
        encoderDrive(10);
        turnRight(135);
        encoderDrive(20);
        plopThePurplePixel();

    }

}