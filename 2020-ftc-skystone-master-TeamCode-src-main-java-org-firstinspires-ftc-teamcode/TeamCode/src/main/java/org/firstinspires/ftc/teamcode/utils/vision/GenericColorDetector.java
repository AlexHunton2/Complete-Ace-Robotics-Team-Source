package org.firstinspires.ftc.teamcode.utils.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;


import java.util.ArrayList;
import java.util.List;

/**
 * Generic Color Detector inspired by Endercv ExampleBlueVision.java
 */

public class GenericColorDetector extends OpenCVPipeline
{
    private boolean showContours = true;

    // Declare the Mats here and reuse them
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();

    // list of contours
    private List<MatOfPoint> contours = new ArrayList<>();

    // the range between 2 colors.
    // Warning HSV in OpenCV is different than in Photoshop: Convert HSV Photoshop Range H:0-360, S: 0-100, V:0-100 to OpenCV Range: H:0-180, S:0-255, V:0-255.
    // Use PhotoshopHSVScalar class to convert or Grip
    private Scalar minColor, maxColor;


    public GenericColorDetector(Scalar min, Scalar max)
    {
        super();
        minColor = min;
        maxColor = max;
    }

    public synchronized void setShowContours(boolean enabled)
    {
        showContours = enabled;
    }

    public synchronized List<MatOfPoint> getContours()
    {
        return contours;
    }

    // This is called every camera frame.

    @Override
    public Mat processFrame(Mat rgba, Mat gray)
    {
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

        // Then we blur to remove noise
        Mat blurInput = hsv;
        double blurRadius = 25.135; // Use GRIP to find this value.
        VisionUtils.blur(blurInput, VisionUtils.BlurType.BOX, blurRadius, hsv);

        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the color in the specified range of values. Use GRIP to find these values.
        Core.inRange(hsv, minColor, maxColor, thresholded);

        // we blur the thresholded image to remove noise
        Imgproc.blur(thresholded, thresholded, new Size(3, 3));

        // create a list to hold our contours.
        // There is going to be a single contour for the outline of every blue object
        // that we can find. We can iterate over them to find objects of interest.
        // the Imgproc module has many functions to analyze individual contours by their area, avg position, etc.
        contours = new ArrayList<>();

        // this function fills our contours variable with the outlines of objects we found
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Then we display our nice little binary threshold on screen
        if (showContours)
        {
            // this draws the outlines of the contours over our original image.
            // they are highlighted in green.
            Imgproc.drawContours(rgba, contours, -1, new Scalar(0, 255, 0), 2, 8);
        }

        return rgba; // display the image seen by the camera
    }


}
