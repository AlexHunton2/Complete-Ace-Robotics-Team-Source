package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.DoNothing;
import org.firstinspires.ftc.teamcode.autonomous.DriveBackward;
import org.firstinspires.ftc.teamcode.autonomous.DriveForward;
import org.firstinspires.ftc.teamcode.autonomous.DriveForwardPID;
import org.firstinspires.ftc.teamcode.autonomous.DrivePID;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotate;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotateGyro;
import org.firstinspires.ftc.teamcode.autonomous.DriveSideways;
import org.firstinspires.ftc.teamcode.autonomous.FoundationGrabberCMD;
import org.firstinspires.ftc.teamcode.autonomous.TapeMeasure;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;


@Autonomous
public class BlueGrabBaseAuto extends AbstractRobotMode {

    private SkystoneDetector detector;
    private FiniteStateMachine fsm;
    Elevator elevator;


    /** This method will be called once when the INIT button is pressed. */
    @Override
    public void init() {
        Dashboard.trace(getClass(), "init() enter");
        super.init();
        fsm = new FiniteStateMachine();
        fsm.addCommand(new DoNothing(10));
        fsm.addCommand(new FoundationGrabberCMD(false));
        fsm.addCommand(new DrivePID(47-18, .5, DriveTrain.RobotDirection.BACKWARD));
        fsm.addCommand(new DrivePID(10, .5, DriveTrain.RobotDirection.RIGHT));
        fsm.addCommand(new DrivePID(3, .5, DriveTrain.RobotDirection.BACKWARD));
        fsm.addCommand(new FoundationGrabberCMD(true));
        fsm.addCommand(new DoNothing(2));
        fsm.addCommand(new DriveForwardPID(47-18+4, .3));
        fsm.addCommand(
                0.0, new DriveRotateGyro(DriveTrain.RobotDirection.ROTATE_RIGHT, .5, 90),
                0.0, new TapeMeasure(Intake.tapeActions.OUT)
        );
        fsm.addCommand(new DrivePID(20, 1, DriveTrain.RobotDirection.BACKWARD));
        fsm.addCommand(new FoundationGrabberCMD(false));
        fsm.addCommand(new DrivePID(10, .5, DriveTrain.RobotDirection.RIGHT));
        fsm.addCommand(new DoNothing(20));
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
