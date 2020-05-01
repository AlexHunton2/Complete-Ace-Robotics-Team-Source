package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

//
public class IMU  {

    //imu = expansion hub
    private BNO055IMU imu;
    private double angleZ = .0; // the one we care
    private double angleY = .0;
    private double angleX = .0;

    public IMU() {
        // Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = AbstractRobotMode.getHardwareMap().get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public boolean isGyroCalibrated()
    {
        return imu.isGyroCalibrated();
    }


    private void updateAngles()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        angleZ = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        angleY = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
        angleX = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle);
    }

    public double getAngleZ()
    {
        updateAngles();
        return angleZ;
    }

    public double getAngleY()
    {
        updateAngles();
        return angleY;
    }

    public double getAngleX()
    {
        updateAngles();
        return angleX;
    }
}
