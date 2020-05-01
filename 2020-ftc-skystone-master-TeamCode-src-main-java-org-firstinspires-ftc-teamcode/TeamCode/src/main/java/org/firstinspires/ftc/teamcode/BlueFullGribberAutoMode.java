package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.autonomous.AlignSidewaysWithSkystone;
import org.firstinspires.ftc.teamcode.autonomous.DetectSkystonePosition;
import org.firstinspires.ftc.teamcode.autonomous.DoNothing;
import org.firstinspires.ftc.teamcode.autonomous.DriveForwardPID;
import org.firstinspires.ftc.teamcode.autonomous.DriveFromOgPosition;
import org.firstinspires.ftc.teamcode.autonomous.DrivePID;
import org.firstinspires.ftc.teamcode.autonomous.DrivePIDRotate;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotateGyro;
import org.firstinspires.ftc.teamcode.autonomous.DriveWithTime;
import org.firstinspires.ftc.teamcode.autonomous.FoundationGrabberCMD;
import org.firstinspires.ftc.teamcode.autonomous.NormalizeAnglePID;
import org.firstinspires.ftc.teamcode.autonomous.SideGrap;
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
public class BlueFullGribberAutoMode extends AbstractRobotMode {

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
        Strategy.allianceColor = Strategy.AllianceColor.BLUE;
        Intake.getInstance().undeploy();
        FoundationGrabber.getInstance().setFoundationGrabbed(false);
        Grabber.getInstance().setSideGripped(Grabber.SideGripState.UP_UNGRIPPED);

        detector = new SkystoneDetector("BLUE");
        detector.init(getHardwareMap().appContext, CameraViewDisplay.getInstance());  // ActivityViewDisplay.getInstance() give Fullscreen
        detector.enable();

        fsm = new FiniteStateMachine();

//        // Detect skystone position
        fsm.addCommand(new DetectSkystonePosition(detector));
//        // Move straight to align
        //double distanceToStone = Strategy.STONE_FROM_WALL-Strategy.ROBOT_LENGTH;
        //fsm.addCommand(new DriveForward(distanceToStone -1, 1.0));
        fsm.addCommand(
                0.0, new DrivePID(27, .4, DriveTrain.RobotDirection.LEFT),
                0.0, new SideGrap(Grabber.SideGripState.UP_PRIMED)
        );
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new AlignSidewaysWithSkystone(1.0));
        fsm.addCommand(new SideGrap(Grabber.SideGripState.DOWN_GRIPPED));
        fsm.addCommand(
                0.0, new SideGrap(Grabber.SideGripState.UP_GRIPPED),
                0.0, new DrivePID(4, 1, DriveTrain.RobotDirection.RIGHT)
        );
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(42, DriveTrain.RobotDirection.BACKWARD, 1));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(35, DriveTrain.RobotDirection.BACKWARD, 1));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DrivePID(5+1.5, 1, DriveTrain.RobotDirection.LEFT));
        fsm.addCommand(new SideGrap(Grabber.SideGripState.UP_UNGRIPPED));
        fsm.addCommand(new DrivePID(2, 1, DriveTrain.RobotDirection.RIGHT));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(41, DriveTrain.RobotDirection.FORWARD, 1));
        fsm.addCommand(new DrivePID(1, 1, DriveTrain.RobotDirection.RIGHT));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(55+15, DriveTrain.RobotDirection.FORWARD, 1));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DrivePID(30, 30, DriveTrain.RobotDirection.RIGHT));
        fsm.addCommand(
                0.0, new DrivePID(27-1, .4, DriveTrain.RobotDirection.LEFT),
                0.0, new SideGrap(Grabber.SideGripState.UP_PRIMED)
        );
        fsm.addCommand(new SideGrap(Grabber.SideGripState.DOWN_GRIPPED));
        fsm.addCommand(
                0.0, new SideGrap(Grabber.SideGripState.UP_GRIPPED),
                0.0, new DrivePID(3, 1, DriveTrain.RobotDirection.RIGHT)
        );
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(50, DriveTrain.RobotDirection.BACKWARD, 1));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new DriveFromOgPosition(30, DriveTrain.RobotDirection.BACKWARD, 1));
        fsm.addCommand(new NormalizeAnglePID(1));
        fsm.addCommand(new SideGrap(Grabber.SideGripState.UP_UNGRIPPED));
        fsm.addCommand(new TapeMeasure(Intake.tapeActions.OUT));


//        fsm.addCommand(new DrivePID(4, 1, DriveTrain.RobotDirection.LEFT));
//        fsm.addCommand(new SideGrap(Grabber.SideGripState.UP_UNGRIPPED));
//        fsm.addCommand(new DrivePIDRotate(-90, 1));
//        fsm.addCommand(new DrivePID(9, .3, DriveTrain.RobotDirection.BACKWARD));
//        fsm.addCommand(new FoundationGrabberCMD(true));
//        fsm.addCommand(new DoNothing(1));
//        fsm.addCommand(new TapeMeasure(Intake.tapeActions.OUT));
//        fsm.addCommand(new DriveForwardPID(47-18+4, 1));
//        fsm.addCommand(new DrivePIDRotate(90, 1));
//        fsm.addCommand(new DrivePID(20+5, 1, DriveTrain.RobotDirection.BACKWARD));
//        fsm.addCommand(new FoundationGrabberCMD(false));
//        fsm.addCommand(new DrivePID(10, 1, DriveTrain.RobotDirection.RIGHT));
//        fsm.addCommand(new DoNothing(20));







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
        Intake.getInstance();
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
