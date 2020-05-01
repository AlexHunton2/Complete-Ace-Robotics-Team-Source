package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.ControllerToggle;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

@TeleOp
public class AlexAceTeleopMode extends AbstractRobotMode {

    Intake intake;

    ControllerToggle toggle;


    @Override
    public void init() {
        super.init();

        intake = Intake.getInstance();
        toggle = new ControllerToggle(gamepadB);

    }

    @Override
    public void loop() {

        //move drivetrain based on gamepad stick values
        //driveTrain.teleop(gamepadB, telemetry);

        toggle.updateButtonStates();


        if (toggle.getADown()) {
            intake.toggleIntakeInward();
        }

        if (toggle.getYDown()) {
            intake.toggleIntakeOutward();
        }

        /*
        DEBUG CODE FOR ELEVATOR:

         if (toggle.getADown()) {
            elevator.setTicks(elevator.returnPos() + 25);
        } else {
             elevator.motorToPoint(elevator.getTicks(), .2);
         }

         */




        //driveTrain.driveDistance_Forward(3, .2, telemetry);




        // Leave telemetry update at the end
        telemetry.update();
    }


    public void stop()
    {
        Dashboard.trace(getClass(), "stop() enter");
//        DriveTrain.resetInstance();
//        Elevator.resetInstance();
//        FoundationGrabber.resetInstance();
//        Grabber.resetInstance();
        Intake.resetInstance();
        Dashboard.trace(getClass(), "stop() exit");
    }


}
