package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.DriveForwardPID;
import org.firstinspires.ftc.teamcode.autonomous.DriveForwardDist;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@Autonomous
public class Test_DriveDistance extends AbstractRobotMode {
    ElapsedTime globalTimer = new ElapsedTime();
    double SPEED = .4;

    private FiniteStateMachine fsm;

    @Override
    public void init()
    {
        fsm = new FiniteStateMachine();
        fsm.addCommand(new DriveForwardPID(30, .3));
    }

    public void start()
    {
        fsm.resetTime();
    }

    @Override
    public void loop()
    {
        fsm.run();
        Dashboard.addData("Correction", DriveTrain.getInstance().getCorrection());
        // Leave telemetry update at the end
        telemetry.update();
    }

    public void stop()
    {
        DriveTrain.resetInstance();
    }
}
