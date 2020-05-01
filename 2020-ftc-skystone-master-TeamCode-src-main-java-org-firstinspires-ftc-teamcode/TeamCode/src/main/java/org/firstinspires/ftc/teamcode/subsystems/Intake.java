package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.KeonAceTeleopMode;

import java.util.Timer;
import java.util.TimerTask;

import static java.lang.System.gc;

public class Intake {

    private static Intake instance;

    private DcMotor intakeLeftMotor; //0
    private DcMotor intakeRightMotor; //1
    private DcMotor intakeConveyerMotor;
    private Servo intakeDeployServo; // 2 (hub 2)
    private CRServo tapeMeasureServo; // 4 (hub 1)

    public enum tapeActions {OUT, IN, STOP}

    float intakeActiveValue = 1;
    float intakeUnactiveValue = 0;

    private tapeActions tapeAction;



    public enum IntakeDirection {
        OUTWARD,
        INWARD,
        STOPPED
    }

    IntakeDirection currentDirection = IntakeDirection.STOPPED;

    //
    // Singleton
    //

    public static Intake getInstance()
    {
        if(instance == null) instance = new Intake();
        return instance;
    }

    public static void resetInstance()
    {
        if(instance != null) instance.stop();
        instance = null;
        gc();
    }

    //
    // Constructor
    //

    private Intake() {
        intakeLeftMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "intake_motor_left");
        intakeRightMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "intake_motor_right");
        intakeDeployServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "intake_servo_deploy");
        intakeConveyerMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "intake_motor_conveyer");
        tapeMeasureServo = AbstractRobotMode.getHardwareMap().get(CRServo.class, "tape_measure");

        intakeConveyerMotor.setDirection(DcMotor.Direction.REVERSE);

        tapeAction = tapeActions.STOP;
    }


    public void deploy()
    {
        intakeDeployServo.setPosition(KeonAceTeleopMode.SERVO_MAX);
    }
    public void undeploy() { intakeDeployServo.setPosition(KeonAceTeleopMode.SERVO_MIN);}

    public void toggleIntakeOutward() {
        if (currentDirection != IntakeDirection.OUTWARD) {
            setIntakeDirection(IntakeDirection.OUTWARD);
        } else {
            setIntakeDirection(IntakeDirection.STOPPED);
        }
    }

    public void toggleIntakeInward() {
        if (currentDirection != IntakeDirection.INWARD) {
            setIntakeDirection(IntakeDirection.INWARD);
        } else {
            setIntakeDirection(IntakeDirection.STOPPED);
        }
    }

    public void setIntakeDirection(IntakeDirection dir) {
        double motorPowerVal = 0;
        double conveyPowerVal = 0;

        switch (dir) {
            case INWARD:
                motorPowerVal = intakeActiveValue;
                conveyPowerVal = intakeActiveValue;
                currentDirection = IntakeDirection.INWARD;
                Grabber.getInstance().canDetectStone = true;
                break;
            case OUTWARD:
                motorPowerVal = conveyPowerVal = -intakeActiveValue / 2;
                currentDirection = IntakeDirection.OUTWARD;
                Grabber.getInstance().setGrabbed(false);
                Grabber.getInstance().canDetectStone = false;
                break;
            case STOPPED:
                motorPowerVal = conveyPowerVal = intakeUnactiveValue;
                currentDirection = IntakeDirection.STOPPED;
                Grabber.getInstance().canDetectStone = true;
                break;
        }
        intakeConveyerMotor.setPower(conveyPowerVal);
        intakeLeftMotor.setPower(motorPowerVal);

        intakeRightMotor.setPower(motorPowerVal);
        intakeConveyerMotor.setPower(conveyPowerVal);
    }

    public void startIntakeLeftInward() {
        double powerVal = 0;
        powerVal = intakeActiveValue;
        currentDirection = IntakeDirection.INWARD;
        intakeLeftMotor.setPower(powerVal);
    }

    public void startIntakeRightInward() {
        double powerVal = 0;
        powerVal = intakeActiveValue;
        currentDirection = IntakeDirection.INWARD;
        intakeRightMotor.setPower(powerVal);
    }

    public void activatePushServoAndGrab(long duration) {
        final long d = duration;
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        Grabber.getInstance().setGrabbed(true);
                        KeonAceTeleopMode.isAutomated = false;
                    }
                }, d);
            }
        }, duration);
    }

    public void toggleTapMeasure(tapeActions action) {
        if (tapeAction == tapeActions.IN  || tapeAction == tapeActions.OUT) {
            setTapeMeasureServo(tapeActions.STOP);
        } else {
            setTapeMeasureServo(action);
        }
    }

    public void setTapeMeasureServo(tapeActions action) {
        int speed = 0;
        switch (action) {
            case OUT: speed = 1; break;
            case IN: speed = -1; break;
            case STOP: speed = 0; break;
        }
        tapeMeasureServo.setPower(speed);
        tapeAction = action;
    }

    public void stop() {
        intakeLeftMotor.setPower(0);
        intakeRightMotor.setPower(0);
    }
}
