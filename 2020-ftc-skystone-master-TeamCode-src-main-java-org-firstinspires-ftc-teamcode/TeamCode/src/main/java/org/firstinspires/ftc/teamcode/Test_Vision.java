package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.firstinspires.ftc.teamcode.autonomous.DetectSkystonePosition;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;


@Autonomous
public class Test_Vision extends AbstractRobotMode {

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

        fsm = new FiniteStateMachine();
        detector = new SkystoneDetector("BLUE");
        detector.init(getHardwareMap().appContext, CameraViewDisplay.getInstance());  // ActivityViewDisplay.getInstance() give Fullscreen
        detector.enable();


        // Detect skystone position
        fsm.addCommand(new DetectSkystonePosition(detector));



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
