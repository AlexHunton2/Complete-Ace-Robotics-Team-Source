package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AceMotorMethods {
    DcMotor angleMotor;

    public AceMotorMethods(DcMotor motor) {
        angleMotor = motor;
    }

    public void motorToPoint(int value, double power) {
        if (angleMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            int current = angleMotor.getCurrentPosition();
            final int range = 25;
            if (current > value) {
                if (current < value + range || current < value - range)
                    angleMotor.setPower(0);
                else
                    angleMotor.setPower(-power);
            } else {
                if (current > value + range || current > value - range)
                    angleMotor.setPower(0);
                else
                    angleMotor.setPower(power);
            }
        } else if (angleMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            angleMotor.setTargetPosition(value);
            if (angleMotor.getCurrentPosition() > value)
                angleMotor.setPower(-power);
            else
                angleMotor.setPower(power);
        }
    }
}
