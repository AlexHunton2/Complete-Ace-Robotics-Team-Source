package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Test_DriveTrain {
    private DcMotor topLeftMotor; //3
    private DcMotor bottomLeftMotor; //1
    private DcMotor topRightMotor; //2
    private DcMotor bottomRightMotor; //0

    private DcMotor encoderXWheelLeft;
    private DcMotor encoderXWheelRight;
    private DcMotor encoderYWheelBack;

    final static int TICKS_PER_INCH = 305;

    public Test_DriveTrain(HardwareMap hardwareMap) {
        //init motors
        topLeftMotor = hardwareMap.get(DcMotor.class, "motor_top_left");
        bottomLeftMotor = hardwareMap.get(DcMotor.class, "motor_bottom_left");
        topRightMotor = hardwareMap.get(DcMotor.class, "motor_top_right");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "motor_bottom_right");

        //init encoders (using the motor ports, but the encoder connection, not the actual dc motor connection.)
        encoderXWheelLeft = bottomLeftMotor;
        encoderXWheelRight = bottomRightMotor;
        encoderYWheelBack = topRightMotor;

        //resetInstance encoder values at start
        encoderXWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderXWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderYWheelBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set encoders to be recognized as encoders and not motors
        encoderXWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderXWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoderYWheelBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reverse motors (that need it)
        topRightMotor.setDirection(DcMotor.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //set non-movement behavior to brake
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //movement with gamepad for teleop
    public void GamepadMove(Gamepad gamepad, Telemetry t) {

        // Left stick controls movement
        double gamepad_left_stick_x = gamepad.left_stick_x;
        double gamepad_left_stick_y = - gamepad.left_stick_y;

        // Right stick controls rotation
        double gamepad_right_stick_x = gamepad.right_stick_x;
        double gamepad_right_stick_y = gamepad.right_stick_y;

        double r = Math.hypot(gamepad_left_stick_x, gamepad_left_stick_y);
        double robotAngle = Math.atan2(gamepad_left_stick_y, -gamepad_left_stick_x) - Math.PI / 4;

        //calculate individual motor power
        final double tlPower = r * Math.cos(robotAngle) + gamepad_right_stick_x;
        final double trPower = r * Math.sin(robotAngle) - gamepad_right_stick_x;
        final double blPower = r * Math.sin(robotAngle) + gamepad_right_stick_x;
        final double brPower = r * Math.cos(robotAngle) - gamepad_right_stick_x;

        topLeftMotor.setPower(tlPower);
        topRightMotor.setPower(trPower);
        bottomLeftMotor.setPower(blPower);
        bottomRightMotor.setPower(brPower);

        //update driver station telemetry data
        t.addData("TL Motor", tlPower);
        t.addData("TR Motor", trPower);
        t.addData("BL Motor", blPower);
        t.addData("BR Motor", brPower);

        t.addData("Gamepad Left", gamepad_left_stick_x + ", " + gamepad_left_stick_y);
        t.addData("Gamepad Right", gamepad_right_stick_x + ", " + gamepad_right_stick_y);
    }

    void driveDistanceForward(double inches, double speed) {

    }

    int getEncoderXTicks() {
        return (encoderXWheelLeft.getCurrentPosition() + encoderXWheelRight.getCurrentPosition()) / 2;
    }

    double ticksToInches(int ticks) {
        return ticks / TICKS_PER_INCH;
    }
}
