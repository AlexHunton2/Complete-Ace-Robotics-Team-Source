package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;


@Autonomous
public class Test_Angles extends AbstractRobotMode {

    ElapsedTime globalTimer = new ElapsedTime();
    double SPEED = .20;

    @Override
    public void init() {
        Dashboard.trace(getClass(), "init() enter");
        super.init();
        globalTimer.reset();
        DriveTrain.getInstance();
        Dashboard.trace(getClass(), "init() exit");
    }


    public void start()
    {
        Dashboard.trace(getClass(), "start() enter");
        globalTimer.reset();
        Dashboard.trace(getClass(), "start() exit");
    }


    @Override
    public void loop()
    {
        Dashboard.trace(getClass(), "loop() enter");
        double seconds = globalTimer.seconds();
        telemetry.addData("Raw Angle", DriveTrain.getInstance().getRawAngle());

        // Leave telemetry update at the end
        telemetry.update();
        Dashboard.trace(getClass(), "loop() exit");
    }

    public void stop()
    {
        DriveTrain.resetInstance();
    }

}
