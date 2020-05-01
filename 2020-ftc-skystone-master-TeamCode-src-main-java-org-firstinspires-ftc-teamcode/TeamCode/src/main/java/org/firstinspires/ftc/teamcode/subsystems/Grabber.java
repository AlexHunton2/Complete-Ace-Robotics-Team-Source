package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.ftccommon.configuration.EditLegacyModuleControllerActivity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.KeonAceTeleopMode;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Timer;
import java.util.TimerTask;

import static java.lang.System.gc;
import static org.firstinspires.ftc.teamcode.KeonAceTeleopMode.redServoDegree;

public class Grabber {

    private static Grabber instance;

    private Servo grabServo; //4 HUB 2
    private Servo spinServo; //5 HUB 2

    private Servo releaseServo; //0 HUB 1
    private Servo clampServo; //1 HUB 1

    public enum SideGripState {UP_GRIPPED, DOWN_GRIPPED, UP_UNGRIPPED, DOWN_UNGRIPPED, UP_PRIMED }

    AnalogInput limitSwitch;
    ColorSensor colorSensor;

    private boolean isGrabbed;

    private SideGripState isSideGripped;
    private boolean isReleased;
    private boolean isClamped;
    private boolean isSpun;

    public boolean canDetectStone = true;

    private int memoryLevel;
    private final int startLevel = 2;


    double grabbedAngle = redServoDegree(180);
    //double grabbedAngle = 270;
    double ungrabbedAngle = redServoDegree(0);

    double spunAngle = redServoDegree(180+7+13);
    double unspunAngle = redServoDegree(0+7);

    float releaseAngle = (float)redServoDegree(0); //DOWN
    float unreleaseAngle = (float)redServoDegree(90+2); //UP

    float clampAngle = (float)redServoDegree(90); //CLAMPED
    float unclampAngle = (float)redServoDegree(30); //UNCLAMPED
    float clampPrime = (float)redServoDegree(5); // PRIMED



    //in ms
    long grabberTimout = 1500;
    float colorSensorThreshhold = 800;

    long grippedDelay = 700;
    long ungrippedDelay = 700;

    //
    // Singleton
    //

    public static Grabber getInstance()
    {
        if(instance == null) instance = new Grabber();
        return instance;
    }

    public static void resetInstance()
    {
        Grabber.getInstance().setGrabbed(false);
        instance = null;
        gc();
    }

    //
    // Constructor
    //

    private Grabber() {
        grabServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "grabber_servo_grab");
        spinServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "grabber_servo_spin");

        releaseServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "grabber_side_servo_release");
        clampServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "grabber_side_servo_clamp");

        memoryLevel = startLevel;

        setSpun(false);
        setGrabbed(false);
        setRelease(false);
        setClamp(true);
        //setSideGripped(SideGripState.UP_UNGRIPPED);
    }

    public Servo getGrabServo() { return grabServo; }


    public void toggleGrab() {
        if (isGrabbed) {
            setGrabbed(false);
        } else {
            setGrabbed(true);
        }
    }

    public void toggleRelease() {
        boolean release = isReleased ? false : true;
        setRelease(release);
    }

    public void toggleClamp() {
        boolean clamp = isClamped ? false : true;
        setClamp(clamp);
    }

    public void setRelease(boolean isRelease) {
        float pos = isRelease ? releaseAngle : unreleaseAngle;
        releaseServo.setPosition(pos);
        isReleased = isRelease;
    }

    public void setClamp(boolean isClamp) {
        float pos = isClamp ? clampAngle : unclampAngle;
        clampServo.setPosition(pos);
        isClamped = isClamp;
    }

    public void setPrimed(boolean isPrimed) {
        float pos = isPrimed ? clampPrime : unclampAngle;
        clampServo.setPosition(pos);
    }

    public void setGrabbed(boolean isGrab) {
        double pos = isGrab ? grabbedAngle : ungrabbedAngle;
        isGrabbed = isGrab;
        grabServo.setPosition(pos);
    }

    public void setSpun(boolean spun) {
        double pos = spun ? spunAngle : unspunAngle;
        isSpun = spun;
        spinServo.setPosition(pos);
    }

    public void setSpunWithElevator(boolean spun) {
        final double pos = spun ? spunAngle : unspunAngle;
        isSpun = spun;
        if (spun) {
            Elevator.getInstance().setLevel(memoryLevel);
            if (!(memoryLevel >= Elevator.getInstance().numberOfLevels)) {
                memoryLevel++;
            } else {
                memoryLevel = startLevel;
            }
            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    spinServo.setPosition(pos);
                }
            }, 1000);
        } else {
            Elevator.getInstance().setLevel(1);
            if (Utils.inRange(Elevator.getInstance().getTicks(), -5, 5)) {
                spinServo.setPosition(pos);
                setGrabbed(false);
            }
        }
    }

    public void toggleSpun() {
        if (isSpun) {
            setSpun(false);
        } else {
            setSpun(true);
        }
    }

    public void setSideGripped(SideGripState isSideGrip) {
        switch (isSideGrip) {
            case UP_GRIPPED: {
                setClamp(true);
                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        setRelease(false);
                    }
                }, grippedDelay);
                break;
            }
            case DOWN_GRIPPED: {
                    setRelease(true);
                    new Timer().schedule(new TimerTask() {
                        @Override
                        public void run() {
                            setClamp(true);
                        }
                    }, grippedDelay);
                    break;
            }
            case UP_UNGRIPPED: {
                setClamp(false);
                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        setRelease(false);
                    }
                }, grippedDelay);
                break;
            }
            case DOWN_UNGRIPPED: {
                setRelease(true);
                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        setClamp(false);
                    }
                }, grippedDelay);
                break;
            }
            case UP_PRIMED: {
                setPrimed(true);
                new Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        setRelease(false);
                    }
                }, grippedDelay);
                break;
            }
        }
        isSideGripped = isSideGrip;
    }

    public SideGripState getSideGripped() {
        return isSideGripped;
    }

    public boolean getGrabbed() {
        return isGrabbed;
    }

}
