package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.autonomous.DetectSkystonePosition;
import org.firstinspires.ftc.teamcode.autonomous.DriveBackward;
import org.firstinspires.ftc.teamcode.autonomous.DriveForward;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotate;
import org.firstinspires.ftc.teamcode.autonomous.DriveSideways;
import org.firstinspires.ftc.teamcode.autonomous.DriveSidewaysFromOrigin;
import org.firstinspires.ftc.teamcode.autonomous.EjectAStone;
import org.firstinspires.ftc.teamcode.autonomous.GrabAStone;
import org.firstinspires.ftc.teamcode.autonomous.AlignSidewaysWithSkystone;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;


@Autonomous
public class BlueAutoMode extends AbstractRobotMode {

    private SkystoneDetector detector;
    private FiniteStateMachine fsm;


    /** This method will be called once when the INIT button is pressed. */
    @Override
    public void init() {
        Dashboard.trace(getClass(), "init() enter");
        super.init();
        Strategy.allianceColor = Strategy.AllianceColor.BLUE;

        fsm = new FiniteStateMachine();
        detector = new SkystoneDetector("BLUE");
        detector.init(getHardwareMap().appContext, CameraViewDisplay.getInstance());  // ActivityViewDisplay.getInstance() give Fullscreen
        detector.enable();


        // Detect skystone position
        fsm.addCommand(new DetectSkystonePosition(detector));
        // Move straight to align
        fsm.addCommand(new AlignSidewaysWithSkystone(1.0));
        double distanceToStone = Strategy.STONE_FROM_WALL-Strategy.ROBOT_LENGTH;
        fsm.addCommand(new DriveForward(distanceToStone -1, 1.0));
        fsm.addCommand(new GrabAStone());
        fsm.addCommand(new DriveBackward(distanceToStone-7 +3 +7, 1.0));
        fsm.addCommand(new DriveSidewaysFromOrigin(-2, DriveTrain.RobotDirection.LEFT, 1.0));
        fsm.addCommand(new EjectAStone());
        fsm.addCommand(new DriveSidewaysFromOrigin(Strategy.STONE_WIDTH*2+7, DriveTrain.RobotDirection.RIGHT, 1.0));

        // try to pickup the 2nd
        fsm.addCommand(new DriveForward(14 +3, 1.0));
        fsm.addCommand(new GrabAStone(true));
        fsm.addCommand(new DriveBackward(distanceToStone +3 +5, 1.0));

        fsm.addCommand(new DriveSidewaysFromOrigin(Strategy.STONE_WIDTH*2+5, DriveTrain.RobotDirection.LEFT, 1.0));
        fsm.addCommand(new EjectAStone());
        fsm.addCommand(new DriveSideways(20, DriveTrain.RobotDirection.RIGHT, 1.0));


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
        fsm.run();

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
