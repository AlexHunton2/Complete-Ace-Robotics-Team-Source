package org.firstinspires.ftc.teamcode.utils.vision;

import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class VisionUtils
{
    // type of filter to use for a blur.
    public enum BlurType{ BOX, GAUSSIAN, MEDIAN, BILATERAL};


    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    public static void blur(Mat input, BlurType type, double doubleRadius, Mat output)
    {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    public static Scalar computeAverageColorOfRectangle(Mat image, Point topleft, Point bottomright)
    {
        Dashboard.debug("computeAverageColorOfRectangle(Mat, " + topleft + ", " + bottomright + ")");
        // show rectangle and extract from image
        Mat subRectangle = image.submat(new Rect(topleft, bottomright));
        Dashboard.debug(" extractRectangle(...)");
        // average its color
        Scalar averageColor = Core.mean(subRectangle);
        Dashboard.debug(" Core.mean(...) returns Scalar="+averageColor);
        // display detection zone
        Imgproc.rectangle(image, topleft, bottomright, new Scalar(0,255,0),1,8,0);
        Imgproc.circle(image, center(topleft, bottomright), (int)((bottomright.y-topleft.y)*.5), averageColor, -1);
        // release the Mat otherwise app WILL crash (fix memory pb)
        Dashboard.debug(" Release the Mat");
        subRectangle.release();
        return averageColor;
    }

    public static Mat extractRectangle(Mat image, Point topleft, Point bottomright)
    {
        return image.submat(new Rect(topleft, bottomright));
    }

    public static Point center(Point p1, Point p2)
    {
        return new Point((p1.x+p2.x)*.5, (p1.y+p2.y)*.5);
    }

    public static double distanceBetweenColors(Scalar color1, Scalar color2)
    {
        return (
            Math.pow(color1.val[0]-color2.val[0], 2)
          + Math.pow(color1.val[1]-color2.val[1], 2)
          + Math.pow(color1.val[2]-color2.val[2], 2)
          + Math.pow(color1.val[3]-color2.val[3], 2));
    }
}
