package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.Utils;


@Autonomous
public class Test_Wheels_With_Distance extends AbstractRobotMode {

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
        Dashboard.banner();
        Dashboard.trace(getClass(), "start() exit");
    }


    @Override
    public void loop()
    {
        Dashboard.trace(getClass(), "loop() enter");
        double seconds = globalTimer.seconds();


        if(Utils.inRange(seconds, 0 , 5))
            DriveTrain.getInstance().driveDistance("commandID1", 20, 0.40, DriveTrain.RobotDirection.FORWARD);
        else if(Utils.inRange(seconds, 5 , 10))
            DriveTrain.getInstance().driveDistance("commandID2", 20, 0.40, DriveTrain.RobotDirection.BACKWARD);
        else if(Utils.inRange(seconds, 10 , 15))
            DriveTrain.getInstance().driveDistance("commandID3", 40, 0.40, DriveTrain.RobotDirection.FORWARD);
        else if(Utils.inRange(seconds, 15 , 20))
            DriveTrain.getInstance().driveDistance("commandID4", 20, 0.40, DriveTrain.RobotDirection.BACKWARD);


        // Leave telemetry update at the end
        telemetry.update();
        Dashboard.trace(getClass(), "loop() exit");
    }

    public void stop()
    {
        DriveTrain.resetInstance();
    }

}
