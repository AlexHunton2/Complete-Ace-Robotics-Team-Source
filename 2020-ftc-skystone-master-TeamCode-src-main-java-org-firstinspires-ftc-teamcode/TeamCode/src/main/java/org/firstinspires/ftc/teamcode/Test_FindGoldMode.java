package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.utils.vision.GenericColorDetector;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

@TeleOp(name="TEST Gold Detector")
public class Test_FindGoldMode extends OpMode {

    private int LEFT_POSITION = 70;
    private int MIDDLE_POSITION = 270;
    private int RIGHT_POSITION = 420;

    private int MINERAL_LEFT_POSITION_LIMIT = (LEFT_POSITION + MIDDLE_POSITION)/2;
    private int MINERAL_RIGHT_POSITION_LIMIT = (MIDDLE_POSITION + RIGHT_POSITION)/2;

    /** Use GRIP to determine values */
    private Scalar goldMinScalar = new Scalar( 0, 193, 76);
    private Scalar goldMaxScalar = new Scalar(28, 255, 255);


    private GenericColorDetector colorDetector;


    @Override
    public void init() {
        colorDetector = new GenericColorDetector(goldMinScalar, goldMaxScalar);
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        colorDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        colorDetector.setShowContours(true);
        // start the vision system
        colorDetector.enable();
    }

    @Override
    public void loop() {
        // update the settings of the vision pipeline
        colorDetector.setShowContours(true);

        // get a list of contours from the vision system
        List<MatOfPoint> contours = colorDetector.getContours();
        for (int i = 0; i < contours.size(); i++) {
            // get the bounding rectangle of a single contour, we use it to get the x/y center
            // yes there's a mass center using Imgproc.moments but w/e
            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
            telemetry.addData("contour" + Integer.toString(i),
                    String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));

            double position = (boundingRect.y + boundingRect.height) / 2;
            telemetry.addData("position:",
                    ( position < MINERAL_LEFT_POSITION_LIMIT ? "LEFT"
                            : position > MINERAL_RIGHT_POSITION_LIMIT ? "RIGHT"
                            : "CENTER"));
            ;
        }
    }

    public void stop() {
        // stop the vision system
        colorDetector.disable();
    }
}