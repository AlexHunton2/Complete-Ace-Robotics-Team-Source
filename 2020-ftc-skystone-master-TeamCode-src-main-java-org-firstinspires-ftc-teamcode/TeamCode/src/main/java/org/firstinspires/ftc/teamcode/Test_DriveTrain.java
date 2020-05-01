package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.Utils;


@Autonomous
public class Test_DriveTrain extends AbstractRobotMode {

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

        if(Utils.inRange(seconds, 0 , 2))
            DriveTrain.getInstance().driveForwardNormal();
        else if(Utils.inRange(seconds, 4 , 6))
            DriveTrain.getInstance().driveBackwardNormal();
        else if(Utils.inRange(seconds, 7 , 9))
            DriveTrain.getInstance().driveSlideLeftNormal();
        else if(Utils.inRange(seconds, 11 , 13))
            DriveTrain.getInstance().driveSlideRightNormal();
        else if(Utils.inRange(seconds, 14 , 16))
            DriveTrain.getInstance().driveRotateLeftNormal();
        else if(Utils.inRange(seconds, 17 , 19))
            DriveTrain.getInstance().driveRotateRightNormal();
        else
            DriveTrain.getInstance().drive(0);

        // Leave telemetry update at the end
        telemetry.update();
        Dashboard.trace(getClass(), "loop() exit");
    }



    public void stop()
    {
        DriveTrain.resetInstance();
    }

}
