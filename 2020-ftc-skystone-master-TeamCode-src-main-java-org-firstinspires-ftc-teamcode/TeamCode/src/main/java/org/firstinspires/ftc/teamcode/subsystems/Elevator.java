package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import org.firstinspires.ftc.teamcode.KeonAceTeleopMode;
import org.firstinspires.ftc.teamcode.utils.Dashboard;

import static java.lang.System.gc;

public class Elevator {

    private static Elevator instance;

    private DcMotor elevatorMotor; //2
    private Servo capstoneServo; // 3 hub 2
    private boolean capstoneServoActive;
    private int level = 0;

    private boolean up = false;
    //the distance between each preset elevator level
    private float levelSpacing;
    private int levels[] = {0, 1200, 2400, 3600, 4800, 6000, 8400, 9600, -300};
    private int ticks;
    int numberOfLevels = levels.length - 2;
    double elevatorPower = 1;
    int elevatorUnbonkLevel = 1;
    int elevatorBonkLevel = 0;


    //
    // Singleton
    //

    public static Elevator getInstance()
    {
        if(instance == null) instance = new Elevator();
        return instance;
    }

    public static void resetInstance()
    {
        if(instance != null) instance.resetAngleMotor();
        instance = null;
        gc();
    }

    //
    // Constructor
    //

    private Elevator() {
        elevatorMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "elevator_motor_right");
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);

        capstoneServo = AbstractRobotMode.getHardwareMap().get(Servo.class, "elevator_servo_capstone");
        setCapstoneServoActivated(true);
        //angleMotorRight.setDirection(DcMotor.Direction.REVERSE);

        resetAngleMotor();

        //encoder value doesn't resetInstance, so this will see what level it should be.

    }

    private void setMotorsPower(double power) {
        elevatorMotor.setPower(power);
    }

    private void setMotorsTarget(int ticks) {
        elevatorMotor.setTargetPosition(ticks);
    }

    public int returnPos() { return elevatorMotor.getCurrentPosition(); }

    public int getLevel() { return  level; }

    public void setLevel(int l) { level = l; }

    public void setTicks(int i) { ticks = i; }

    public int getTicks() { return  ticks; }

    public DcMotor.RunMode getMode() { return elevatorMotor.getMode(); }

    public boolean getUp() {
        return up;
    }

    public void resetAngleMotor() {
        elevatorMotor.setPower(0);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    public void setUp(boolean v) {
        up = v;
    }

    //elevator has 7 preset level lengths
    public void showAngle(Telemetry t) {
        t.addData("Current Pos", elevatorMotor.getCurrentPosition());
        t.addData("Goal Pos", elevatorMotor.getTargetPosition());
        t.addData("mode", elevatorMotor.getMode());
    }

    public void motorToPoint(int value, double power) {
        if (elevatorMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            resetAngleMotor();
        } else if (elevatorMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            //angleMotor.setTargetPosition(value);
            setMotorsTarget(value);
            if (elevatorMotor.getCurrentPosition() > value)
                //angleMotor.setPower(-power);
                setMotorsPower(-power);
            else
                //angleMotor.setPower(power);
                setMotorsPower(power);
        } else {
            resetAngleMotor();
        }
    }
    public void moveToLevel(int level) {
        Dashboard.addData("Elevator Encoder", elevatorMotor.getCurrentPosition());
        switch (level) {
            case 0: motorToPoint(levels[0], elevatorPower); break;
            case 1: motorToPoint(levels[1], elevatorPower); break;
            case 2: motorToPoint(levels[2], elevatorPower); break;
            case 3: motorToPoint(levels[3], elevatorPower); break;
            case 4: motorToPoint(levels[4], elevatorPower); break;
            case 5: motorToPoint(levels[5], elevatorPower); break;
            case 6: motorToPoint(levels[6], elevatorPower); break;
            case 7: motorToPoint(levels[7], elevatorPower); break;
            case 8: motorToPoint(levels[8], elevatorPower); break;
            case 69: motorToPoint(levels[9], elevatorPower); break;
        }
    }

    public void incrementLevel() {
        if (getLevel() < 8)
            setLevel(getLevel() + 1);
        else
            setLevel(elevatorUnbonkLevel);
    }

    public void decrementLevel() {
        if (getLevel() > elevatorUnbonkLevel) {
            setLevel(getLevel() - 1);
        } else {
            setLevel(0);
        }
    }

    public void toggleCapstone() {
        if (!capstoneServoActive) {
            setCapstoneServoActivated(true);
        } else {
            setCapstoneServoActivated(false);
        }
    }

    public void setCapstoneServoActivated(boolean isActive) {
        float val = isActive ? KeonAceTeleopMode.SERVO_MIN : KeonAceTeleopMode.SERVO_MAX;
        capstoneServo.setPosition(val);
        capstoneServoActive = isActive;
    }

    public void bonk() {
        setLevel(elevatorBonkLevel);
    }
}
