package org.firstinspires.ftc.teamcode.utils.vision;

import org.opencv.core.Scalar;


public class PhotoshopHSVScalar extends Scalar
{

	/** Convert HSV Photoshop Range H:0-360, S: 0-100, V:0-100 to OpenCV Range: H:0-180, S:0-255, V:0-255. */
	public PhotoshopHSVScalar(double hueInPS, double saturationInPs, double valueInPs)
	{
		super(	hueInPS/2.00, 
				saturationInPs/2.55, 
				valueInPs*2.55,
				1.00);
	}
	
	public PhotoshopHSVScalar(Scalar hsvPhotoshop)
	{
		super(hsvPhotoshop.val[0]/2.00,
			  hsvPhotoshop.val[1]*2.55,
			  hsvPhotoshop.val[2]*2.55,
			  hsvPhotoshop.val[3]*1.00);
	}

	public double getHue()
	{
		return val[0];
	}

	public double getSaturation()
	{
		return val[1];
	}

	public double getValue()
	{
		return val[2];
	}
}
