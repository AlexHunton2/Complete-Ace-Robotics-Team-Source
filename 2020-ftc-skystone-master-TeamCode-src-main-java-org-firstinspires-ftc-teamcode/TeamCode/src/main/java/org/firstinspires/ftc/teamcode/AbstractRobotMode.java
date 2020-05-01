package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

//
public abstract class AbstractRobotMode extends OpMode {

    protected Gamepad gamepadA, gamepadB = null;

    // Make hardwareMap static to be accessible from any subsystem
    private static HardwareMap hardwareMap;


    @Override
    public void init() {
        Dashboard.trace(getClass(),": init() enter");
        Dashboard.setTelemetry(telemetry);

        // Make hardwareMap static to be accessible from any subsystem
        hardwareMap = super.hardwareMap;
        Dashboard.trace(getClass(),"hardwareMap="+ hardwareMap);

        gamepadA = gamepad1; // renaming the gamepad to match////
        gamepadB = gamepad2;

        Dashboard.addData("Reverse topRightMotor:", "No");
        Dashboard.addData("Reverse bottomLeftMotor:", "No");
        Dashboard.trace(this.getClass(),"init() exit");

    }

    public static HardwareMap getHardwareMap()
    {
        return hardwareMap;
    }

    public void stop()
    {
//        Dashboard.trace(getClass(), "stop() enter");
        DriveTrain.resetInstance();
        Grabber.getInstance().resetInstance();
//        Dashboard.trace(getClass(), "stop() exit");
    }

}
