package org.firstinspires.ftc.teamcode.utils.vision;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * Skystone Detector based on endercv
 */

public class SkystoneDetector extends OpenCVPipeline
{
    // Declare the Mats here and reuse them
    private Mat subRectangle = new Mat();

    private Scalar skystoneColor;
    private Scalar averageColor1, averageColor2, averageColor3, averageColor4;

    private String fieldColor;

    private int skystonePosition = -1;



    public SkystoneDetector(String color)
    {
        super();
        fieldColor = color;
        skystoneColor = new Scalar(0,0,0,0);
    }

    // This is called every camera frame.

    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        //default color to blue

        Dashboard.debug("processFrame: Start");
        float redOffset = 150;
        int top = -20; //px
        int height = (int)(rgba.height()/5.5); // how many cubes fit in one image?
        int xmin = (int)(rgba.width()*0.58);  // crop to not see above the cubes
        int xmax = (int)(rgba.width()*0.67); // crop the top to avoid the carpet
        int y_padding = rgba.height()/15; // spacing between rect

        Dashboard.debug("Mat size is " + rgba.width() + "x" + rgba.height());
        Dashboard.debug("Slice size is " + height+ "x" + (xmax-xmin));


        // TODO: do that in a loop
//        averageColor1 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,height*0), new Point(xmax, height*1-y_padding));
        if (fieldColor == "RED") {
            Dashboard.addData("SIDE", "RED");
            averageColor2 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*-1), new Point(xmax, top+height*-2-y_padding));
            averageColor3 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*-2), new Point(xmax, top+height*-3-y_padding));
            averageColor4 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*-3), new Point(xmax, top+height*-4-y_padding));
        } else if (fieldColor == "BLUE") {
            Dashboard.addData("SIDE", "BLUE");
            averageColor2 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*1), new Point(xmax, top+height*2-y_padding));
            averageColor3 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*2), new Point(xmax, top+height*3-y_padding));
            averageColor4 = VisionUtils.computeAverageColorOfRectangle(rgba, new Point(xmin,top+height*3), new Point(xmax, top+height*4-y_padding));
        }

        double mindistance = Double.MAX_VALUE;
        int localSkystonePosition = -1;

        double distances[] = {
//                VisionUtils.distanceBetweenColors(skystoneColor, averageColor1),
                VisionUtils.distanceBetweenColors(skystoneColor, averageColor2),
                VisionUtils.distanceBetweenColors(skystoneColor, averageColor3),
                VisionUtils.distanceBetweenColors(skystoneColor, averageColor4)
        };

        for(int i=0; i < distances.length; i++)
        {
            if (distances[i] >= mindistance) continue;
            mindistance = distances[i];
            localSkystonePosition = i;
        }

        // correct order: depends on phone orientation
        skystonePosition = distances.length-localSkystonePosition;

        // Display text
        Imgproc.putText(rgba,        //Matrix obj of the image
                "Position " + skystonePosition,           //Text to be added
                new Point(10, 390),        //point
                Core.FONT_HERSHEY_SIMPLEX , //front face
                1,                          //front scale
                new Scalar(0, 255, 0),        //Scalar object for color
                4);                         //Thickness


        Dashboard.debug("processFrame: End");
        return rgba; // display the image seen by the camera
    }

    public int getSkystonePosition()
    {
        return skystonePosition;
    }

}
