package org.firstinspires.ftc.teamcode.utils.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;


public class Detector2018_GoldMineral {

    private int LEFT_POSITION = 70;
    private int MIDDLE_POSITION = 270;
    private int RIGHT_POSITION = 420;

    private int MINERAL_LEFT_POSITION_LIMIT = (LEFT_POSITION + MIDDLE_POSITION)/2;
    private int MINERAL_RIGHT_POSITION_LIMIT = (MIDDLE_POSITION + RIGHT_POSITION)/2;

    private GenericColorDetector colorDetector;
    /** Use GRIP to determine values */
    private Scalar goldMinScalar = new Scalar( 0, 193, 76);
    private Scalar goldMaxScalar = new Scalar(28, 255, 255);

    public enum Position { UNKNOWN, LEFT, CENTER, RIGHT };
    public Position position = Position.UNKNOWN;

    OpMode opmode;


    public Detector2018_GoldMineral(OpMode opmode)
    {
        this.opmode = opmode;

        colorDetector = new GenericColorDetector(goldMinScalar, goldMaxScalar);

        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        colorDetector.init(opmode.hardwareMap.appContext, CameraViewDisplay.getInstance());

        // start the vision system
        colorDetector.enable();
    }


    public Position getPosition()
    {
        int leftCount = 0, centerCount = 0, rightCount = 0, unknownCount = 0;

        // update the settings of the vision pipeline
        colorDetector.setShowContours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = colorDetector.getContours();
        opmode.telemetry.addData("Contours", contours.size());

        for (int i = 0; i < contours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));

            double position = (boundingRect.y + boundingRect.height) / 2;
            if ( position < MINERAL_LEFT_POSITION_LIMIT )
                leftCount++;
            else if ( position > MINERAL_RIGHT_POSITION_LIMIT )
                rightCount++;
            else if ( position > MINERAL_LEFT_POSITION_LIMIT && position < MINERAL_RIGHT_POSITION_LIMIT )
                centerCount++;
            else
                unknownCount++;
        }

        // return the position with the most hits
        int max = Math.max(Math.max(leftCount, rightCount), Math.max(centerCount, unknownCount));
        return
              max == 0              ? Position.UNKNOWN
            : leftCount == max      ? Position.LEFT
            : rightCount  == max    ? Position.RIGHT
            : centerCount  == max   ? Position.CENTER
            : Position.UNKNOWN;
    }


    public void startDetection() {
        // start the vision system
        colorDetector.enable();
    }


    public void stopDetection() {
        // stop the vision system
        colorDetector.disable();
    }
}
