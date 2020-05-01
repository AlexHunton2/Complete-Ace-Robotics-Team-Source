package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.KeonAceTeleopMode;

import static java.lang.System.gc;

public class FoundationGrabber {

    private static FoundationGrabber instance;
    private Servo leftFoundationServo; // 2 Hub 1
    private Servo rightFoundationServo; // 3 Hub 1
    boolean isActive;

    private float leftGrabbedAngle = KeonAceTeleopMode.SERVO_MAX;
    private float leftUngrabbedAngle = KeonAceTeleopMode.SERVO_MIN;

    private float rightGrabbedAngle = KeonAceTeleopMode.SERVO_MIN;
    private float rightUngrabbedAngle = KeonAceTeleopMode.SERVO_MAX;

    //
    // Singleton
    //

    public static FoundationGrabber getInstance()
    {
        if(instance == null) instance = new FoundationGrabber();
        return instance;
    }

    public static void resetInstance()
    {
        instance = null;
        gc();
    }

    //
    // Constructor
    //

    private FoundationGrabber() {
        leftFoundationServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "left_foundation_servo_grab");
        rightFoundationServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "right_foundation_servo_grab");
        setFoundationGrabbed(false);
    }


    public void toggleActive() {
        if (!isActive) {
            setFoundationGrabbed(true);
        } else {
            setFoundationGrabbed(false);
        }
    }

    public void setFoundationGrabbed(boolean grabbed) {
        if (grabbed) {
            leftFoundationServo.setPosition(leftGrabbedAngle);
            rightFoundationServo.setPosition(rightGrabbedAngle);
            isActive = true;
        } else {
            leftFoundationServo.setPosition(leftUngrabbedAngle);
            rightFoundationServo.setPosition(rightUngrabbedAngle);
            isActive = false;
        }
    }

}
