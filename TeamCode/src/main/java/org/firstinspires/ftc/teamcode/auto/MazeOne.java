package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 */
@Autonomous(name="Maze One", group="DanceDemo")
public class MazeOne extends AutoDriveTest {

    @Override
    public void ratCrewGo() {
        encoderDrive(29); //  drives forward X inches
        turnLeft(90); // turns x degrees
        encoderDrive(72); //  drives forward X inches;
        turnRight(90);
        encoderDrive(72);
        turnRight(90);
        encoderDrive(54);
        turnRight(50);
        encoderDrive(60);

        // test change.
    }
}
