package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.DriveForward;
import org.firstinspires.ftc.teamcode.autonomous.Test_Command;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.utils.Dashboard;


@Autonomous
public class Test_StateMachine extends AbstractRobotMode {

    private FiniteStateMachine fsm;

    @Override
    public void init()
    {
        Dashboard.setTelemetry(telemetry); // no call to super setTelemetry so I don't need the hub
        fsm = new FiniteStateMachine();
        fsm.addCommand(new DriveForward(30, .30));
        fsm.addCommand(new Test_Command("test1", 5));
        Dashboard.trace(getClass(), "Test_StateMachine.setTelemetry()");
    }

    public void start()
    {
        fsm.resetTime();
    }

    @Override
    public void loop()
    {
        fsm.run();
        // Leave telemetry update at the end
        telemetry.update();
    }


}
