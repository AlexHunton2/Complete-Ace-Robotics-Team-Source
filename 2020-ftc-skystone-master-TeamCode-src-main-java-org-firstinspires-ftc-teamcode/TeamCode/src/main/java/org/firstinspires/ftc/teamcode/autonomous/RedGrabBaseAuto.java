package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;


@Autonomous
public class RedGrabBaseAuto extends AbstractRobotMode {

    private SkystoneDetector detector;
    private FiniteStateMachine fsm;
    Elevator elevator;


    /** This method will be called once when the INIT button is pressed. */
    @Override
    public void init() {
        Dashboard.trace(getClass(), "init() enter");
        super.init();
        elevator = Elevator.getInstance();
        Grabber.getInstance();
        elevator.resetAngleMotor();
        fsm = new FiniteStateMachine();
        fsm.addCommand(new DoNothing(10));
        fsm.addCommand(new FoundationGrabberCMD(true));
        fsm.addCommand(new DriveBackward(47-18, .5));
        fsm.addCommand(new FoundationGrabberCMD(false));
        fsm.addCommand(new DoNothing(1));
        fsm.addCommand(new DriveForward(47-18, .5));
        fsm.addCommand(new FoundationGrabberCMD(true));
        fsm.addCommand(new DriveSideways(36, DriveTrain.RobotDirection.LEFT, 0.5));

        Dashboard.trace(getClass(), "ini() exit");
    }

    /** This method will be called repeatedly when the INIT button is pressed.*/
    public void init_loop()
    {
//        Dashboard.writeTrace(getClass(), "init_loop() enter");
//        Dashboard.writeTrace(getClass(), "init_loop() exit");
    }

    /** This method will be called once when the PLAY button is first pressed. */
    public void start()
    {
        Dashboard.trace(getClass(), "start() enter");
        Dashboard.banner();
        Strategy.robotStartAngle = DriveTrain.getInstance().getAngle();
        fsm.resetTime();
        Dashboard.trace(getClass(), "start() exit");
    }


    @Override
    public void loop()
    {
        Dashboard.trace(getClass(), "loop() enter");
        //elevator.moveToLevel(Elevator.getInstance().getLevel());
        fsm.run();
        telemetry.addData("TrackedTicks", Strategy.trackedTicks);
        Dashboard.addData("Level", Elevator.getInstance().getLevel());
//        Dashboard.addData("Right Ticks", Elevator.getInstance().getRightTicks());
//        Dashboard.addData("RighTick Goal", Elevator.getInstance().getRightMode());
        // Leave telemetry update at the end
        telemetry.update();
        Dashboard.trace(getClass(), "loop() exit");
    }

    public void stop()
    {
        Dashboard.trace(getClass(), "stop() enter");
        DriveTrain.resetInstance();
        Elevator.resetInstance();
        FoundationGrabber.resetInstance();
        Grabber.resetInstance();
        Intake.resetInstance();
        Dashboard.trace(getClass(), "stop() exit");
    }

}
