package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.Utils;


@Autonomous
public class Test_Wheels extends AbstractRobotMode {

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

        // Test each motor for 3 seconds
        double tl = (Utils.inRange(seconds, 0 , 3) || seconds > 13) ? SPEED : 0;
        double tr = (Utils.inRange(seconds, 3 , 6) || seconds > 13)  ? SPEED : 0;
        double bl = (Utils.inRange(seconds, 6 , 9) || seconds > 13)  ? SPEED : 0;
        double br = (Utils.inRange(seconds, 9 , 13) || seconds > 13)  ? SPEED : 0;
        DriveTrain.getInstance().drive(tl, tr , br, bl);

        // Leave telemetry update at the end
        telemetry.update();
        Dashboard.trace(getClass(), "loop() exit");
    }

    public void stop()
    {
        DriveTrain.resetInstance();
    }

}
