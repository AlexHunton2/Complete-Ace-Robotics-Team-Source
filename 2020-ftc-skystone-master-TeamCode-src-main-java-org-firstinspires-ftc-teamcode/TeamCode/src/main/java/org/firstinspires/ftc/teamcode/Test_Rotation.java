package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.DriveForward;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotate;
import org.firstinspires.ftc.teamcode.autonomous.DriveRotateGyro;
import org.firstinspires.ftc.teamcode.autonomous.FoundationGrabberCMD;
import org.firstinspires.ftc.teamcode.statemachine.FiniteStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@Autonomous
public class Test_Rotation extends AbstractRobotMode {
    private FiniteStateMachine fsm;


    /** This method will be called once when the INIT button is pressed. */
    @Override
    public void init() {
        super.init();
        fsm = new FiniteStateMachine();

        fsm.addCommand(new DriveForward(5, .2));


        fsm.addCommand(new DriveRotateGyro(DriveTrain.RobotDirection.ROTATE_RIGHT, .5, 270));
        //fsm.addCommand(new DriveRotateGyro(DriveTrain.RobotDirection.ROTATE_LEFT, .5, 90));

        fsm.addCommand(new DriveRotate(DriveTrain.RobotDirection.ROTATE_RIGHT, .5, 180));
        fsm.addCommand(new DriveRotate(DriveTrain.RobotDirection.ROTATE_RIGHT, .5, 270));
        fsm.addCommand(new FoundationGrabberCMD(true));
        fsm.addCommand(new DriveRotate(DriveTrain.RobotDirection.ROTATE_LEFT, .5, 90));
        fsm.addCommand(new DriveRotate(DriveTrain.RobotDirection.ROTATE_RIGHT, .5, 360));
        fsm.addCommand(new FoundationGrabberCMD(false));
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
        Dashboard.banner();
        Strategy.robotStartAngle = DriveTrain.getInstance().getAngle();
        fsm.resetTime();
    }


    @Override
    public void loop()
    {
        Dashboard.trace(getClass(), "loop() enter");
        fsm.run();

        Dashboard.addData("Angle", DriveTrain.getInstance().getAngle());
        Dashboard.addData("Ticks", DriveTrain.getInstance().getTempTicks());
        Dashboard.addData("Running", DriveTrain.getInstance().isAllRunning());

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
