package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.autonomous.AlignSidewaysWithSkystone;
import org.firstinspires.ftc.teamcode.autonomous.DetectSkystonePosition;
import org.firstinspires.ftc.teamcode.autonomous.DriveBackward;
import org.firstinspires.ftc.teamcode.autonomous.DriveForward;
import org.firstinspires.ftc.teamcode.autonomous.DriveSideways;
import org.firstinspires.ftc.teamcode.autonomous.DriveSidewaysFromOrigin;
import org.firstinspires.ftc.teamcode.autonomous.EjectAStone;
import org.firstinspires.ftc.teamcode.autonomous.GrabAStone;
import org.firstinspires.ftc.teamcode.autonomous.Test_Parallel_EjectAStone;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.statemachine.CommandGroup;
import org.firstinspires.ftc.teamcode.statemachine.CommandListener;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;


@Autonomous
public class Test_ParallelCommands extends AbstractRobotMode {

    private SkystoneDetector detector;
    private FiniteStateMachine fsm;


    /** This method will be called once when the INIT button is pressed. */
    @Override
    public void init() {
        Dashboard.trace(getClass(), "init() enter");
        super.init();
        Strategy.allianceColor = Strategy.AllianceColor.BLUE;

        fsm = new FiniteStateMachine();

        fsm.addCommand(
            0.0, new DriveForward(50, .3),
            3.0, new Test_Parallel_EjectAStone()
        );


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
