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
public class JackpotMode extends AbstractRobotMode {

    DriveTrain driveTrain;

    @Override
    public void init() {
        super.init();
        driveTrain = DriveTrain.getInstance();
    }

    @Override
    public void loop() {
        //keep this at beginning of loop
        driveTrain.teleop(gamepadA);

        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
