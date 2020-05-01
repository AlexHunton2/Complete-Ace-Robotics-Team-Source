package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.ControllerToggle;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@Autonomous
public class AlexAceAutoMode extends AbstractRobotMode {


    private DriveTrain driveTrain;
    //private Intake intake;
    //private Elevator elevator;
    //private Grabber grabber;
    //private FoundationGrabberCMD foundationGrabber;
    //private ControllerToggle toggle;
    //private ColorSensor colorSensor;
    private IMU imu;




    @Override
    public void init() {
        super.init();
        imu = new IMU();
    }

    @Override
    public void loop() {
        Dashboard.addData("Raw Angle", imu.getAngleZ());
        Dashboard.addData("Angle", driveTrain.getInstance().getAngle());
        telemetry.update();


    }


    public void stop()
    {
        Dashboard.trace(getClass(), "stop() enter");
        DriveTrain.resetInstance();
        //Elevator.resetInstance();
        //FoundationGrabberCMD.resetInstance();
        //Grabber.resetInstance();
        //Intake.resetInstance();
        Dashboard.trace(getClass(), "stop() exit");
    }
}
