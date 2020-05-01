package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.utils.vision.GenericColorDetector;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;
import java.util.Locale;

@TeleOp(name="TEST Skystone Detector")
public class Test_FindSkystoneMode extends OpMode {

    private SkystoneDetector detector;

    @Override
    public void init() {
        detector = new SkystoneDetector("BLUE");
        // ActivityViewDisplay.getInstance() give Fullscreen
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        // start the vision system, it is running in the background
        detector.enable();
    }

    @Override
    public void loop() {
        // update the settings of the vision pipeline
    }

    public void stop() {
        // stop the vision system
        detector.disable();
    }
}