package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 */
@Autonomous(name="Pixel Guesser", group="Z")
public class PixelGuesser extends AutoBase {

    @Override
    public void ratCrewGo() {

        ratCrewWaitSecs(2);
        encoderDrive(8);
        encoderDrive(3);
        encoderDrive(-3);

        Recognition gotit = null;
        for (int j = 0 ;(gotit == null) && (j < 2) ; j++) {

            for (int i = 0; (gotit == null) && (i < 2); i++) {
                List<Recognition> recogs = getRecognitions();
                if ((recogs != null) && (recogs.size() > 0)) {
                    gotit = recogs.get(0);
                    break;
                }
                ratCrewWaitSecs(500);
            }

            if (gotit == null) {
                encoderDrive(9);
                continue;
            }

            recognitionsList.clear();
            recognitionsList.add(gotit);
            break;
        }

        ratCrewWaitSecs(30);


        //turnLeft(90);


        //plopThePurplePixel();
    }

}