package org.firstinspires.ftc.teamcode.subsystems;


import android.graphics.Point;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.AbstractRobotMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.HashMap;

import static java.lang.System.gc;

/**              X
 *               ^
 *               |
 *        +--------------+
 *      +-+    Front     +-+
 *     (   )     |      (   )
 *      +-+      |       +-+
 *        |      |       |
 *        |      |       |
 *        |      +---------------> Y
 *        |              |
 *        |              |
 *        |              |
 *      +-+              +-+
 *     (   )            (   )
 *      +-+     Back     +-+
 *        +--------------+
 *
 */
public class DriveTrain {

    private static DriveTrain instance;

    double movementScalar = 1;
    int directionScale = 1;
    boolean slowed;
    float slowedValue = 0.4f;

    private DcMotor topLeftMotor; //3
    private DcMotor bottomLeftMotor; //1
    private DcMotor topRightMotor; //2
    private DcMotor bottomRightMotor; //0

    private IMU imu;

    private DcMotor wheelEnYRIGHT; //3
    private DcMotor wheelEnYLEFT; // 1
    private DcMotor wheelEnXBACK; // 0

    private int EnYRightTotal;
    private int EnYLeftTotal;
    private int EnXBackTotal;

    private Point position;

    public enum RobotDirection {RIGHT, LEFT, FORWARD, BACKWARD, ROTATE_LEFT, ROTATE_RIGHT};

    private int tickGoal;
    private int tickAngGoal;
    private double GyroAngGoal;

    boolean disComp = false;

    private PIDController pidDrive;
    private PIDController pidTicks;
    private PIDController pidRotate;
    private double correction;
    private double lastAngle;
    private double globalAngle;
    private double rotation;

    HashMap commandIDs = new HashMap();

    private boolean safetySwitch = false;


    public boolean getdisComp() { return disComp; }

    public void setdisComp(boolean t) { disComp = t; }


    //
    // Singleton
    //

    public static DriveTrain getInstance()
    {
        if(instance == null) instance = new DriveTrain();
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

    private DriveTrain()
    {
        Dashboard.trace(getClass(),"DriveTrain() enter");
        //setTelemetry motors
        topLeftMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "motor_top_left");
        bottomLeftMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "motor_bottom_left");
        topRightMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "motor_top_right");
        bottomRightMotor = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "motor_bottom_right");

        //init imu
        imu = new IMU();

        wheelEnYLEFT = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "intake_motor_right"); // 1 LEFT
        wheelEnXBACK = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "intake_motor_left"); // 0 BACK
        wheelEnYRIGHT = AbstractRobotMode.getHardwareMap().get(DcMotor.class, "elevator_motor_right"); // 3 RIGHT

        //resetInstance encoders
        wheelEnYRIGHT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelEnYLEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelEnXBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set modes for motors (RUN WITHOUT ENCODERS ON WHEELS!!!)
        wheelEnYRIGHT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelEnYLEFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelEnXBACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reverse motors (that need it) and set non-movement behavior to brake
        Dashboard.trace(getClass(),"DriveTrain() before reversing");
        writeTrace();
        bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        topLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Dashboard.trace(getClass(),"DriveTrain() after reversing");
        writeTrace();
        Dashboard.trace(getClass(),"DriveTrain() exit");
    }

    public int getTempTicks() {
        return topLeftMotor.getCurrentPosition();
    }
    public int getTopLeftTicks() { return topLeftMotor.getCurrentPosition(); }
    public int getTopLeftGoal() { return topLeftMotor.getTargetPosition(); }
    public int getTopRightTicks() { return topRightMotor.getCurrentPosition(); }
    public int getBottomLeftTicks() { return bottomLeftMotor.getCurrentPosition(); }
    public int getBottomRightTicks() { return  bottomRightMotor.getCurrentPosition(); }
    public double getTopLeftSpeed() { return topLeftMotor.getPower(); }
    public double getTopRightSpeed() { return topRightMotor.getPower(); }
    public double getBottomLeftSpeed() { return bottomLeftMotor.getPower(); }
    public double getBottomRightSpeed() { return bottomRightMotor.getPower(); }


    public boolean isRunning(DcMotor motor) {
        return (motor.getPower() != 0 ? true : false);
    }

    public boolean isAllRunning() {
        return (isRunning(topRightMotor) || isRunning(topLeftMotor) || isRunning(bottomRightMotor) || isRunning(bottomLeftMotor));
    }

    public boolean isAllRunToPos() {
        if(topLeftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || topRightMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || bottomLeftMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION || bottomRightMotor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isAllRunEncode() {
        if(topLeftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || topRightMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || bottomLeftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER || bottomRightMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
            return true;
        } else {
            return false;
        }
    }

    private boolean isAllBusy() {
        if (topRightMotor.isBusy() || topLeftMotor.isBusy() || bottomLeftMotor.isBusy() || bottomRightMotor.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isEqualToGoal(int tickGoal, DcMotor motor) {
        if (Utils.inRange(Math.abs(motor.getCurrentPosition()), tickGoal-10, tickGoal+10)) {
            return true;
        } else {
            return false;
        }
    }

    public void safetyProtocal() {
        if (Math.abs(topLeftMotor.getCurrentPosition()) > tickGoal+25) {
            safetySwitch = true;
        } else {
            safetySwitch = false;
        }
    }

    public boolean isAllEqualToGoal(int tickGoal) {
        if (isEqualToGoal(tickGoal, topLeftMotor) || isEqualToGoal(tickGoal, topRightMotor) || isEqualToGoal(tickGoal, bottomRightMotor) || isEqualToGoal(tickGoal, bottomLeftMotor)) {
            return true;
        } else {
            return false;
        }
    }

    public int convertTicksToInches(double distance, RobotDirection direction) {
        int ticksPerInch;
        if (direction == RobotDirection.FORWARD || direction == RobotDirection.BACKWARD) {
            ticksPerInch = 58;
        } else {
            ticksPerInch = 67;
        }
        int ticksDistance = (int)(ticksPerInch*distance);
        return ticksDistance;
    }

    public double convertForTickToInches(double ticks) {
        return ticks / 58;
    }

    public int convertTicksToDegrees(double angle, boolean set, RobotDirection direction) {
        //set variable is used to see if you want to set the angle to a global angel or to add an amount of degrees to the current
        //TRUE: Set
        //False: Add
        int ticksPerAngle = 16; //default value
        double currentangle = getAngle() < 359 ? getAngle() : 0; //fixes bug when robot starts and is like -0.01
        if (direction == RobotDirection.ROTATE_LEFT) {
            angle = set ? Math.abs(angle - currentangle) : angle;
        } else if (direction == RobotDirection.ROTATE_RIGHT){
            angle = set ? Math.abs((currentangle - angle) + 360) : angle;
        }
        return (int) (ticksPerAngle * angle);
    }

    private double convertAngle(double angle) {
        if (angle > 0) {
            return angle;
        } else {
            return Math.abs(angle + 360);
        }
    }

    public double getRawAngle() {
        return imu.getAngleZ();
    }
    public double getCorrection() {return correction;}
    public void resetTickGoal() {tickGoal = 0;}
    public void resetSafety() { safetySwitch = false; }

    private enum CommandStatus { PROCESSED, IN_PROGRESS}
    private boolean isCommandNew(String commandId) { return !commandIDs.containsKey(commandId); }
    private boolean isCommandInprogress(String commandId) { return commandIDs.containsKey(commandId) && commandIDs.get(commandId)==CommandStatus.IN_PROGRESS; }
    private boolean isCommandProcessed(String commandId) { return commandIDs.containsKey(commandId) && commandIDs.get(commandId)==CommandStatus.PROCESSED;  }
    private boolean isCommandAtDestination(String commandId) { return isAllEqualToGoal(tickGoal); }
    private boolean isCommandAtAngle(String commandId) { return isAllEqualToGoal(tickAngGoal); }
    private boolean isCommandAtGyroAngle(String commandId, double currentAng, double goalAng) { return Utils.inRange(currentAng, goalAng-1, goalAng+1);}
    private void setCommandStatus(String commandId, CommandStatus status) { commandIDs.put(commandId, status); }

    /** commandId is an unique identifier of the command */
    public boolean driveDistance(String commandId, double distance, double speed, RobotDirection direction) {
        writeTrace();

        // is it a new command?
        if(isCommandNew(commandId))
        {
            // ...Yes, command is new
            // prepare for moving
            Dashboard.trace(getClass(), "New Command. Initializing motors with new target");
            setCommandStatus(commandId, CommandStatus.IN_PROGRESS);
            tickGoal = convertTicksToInches(distance, direction);
            drivePos_Direction(tickGoal, speed, direction);
            Dashboard.trace(getClass(), "driveDistance() exit");
            return false;
        }

        // ...No, this command is not new
        // but is it an old command already processed?
        if(isCommandProcessed(commandId))
        {
            // ...Yes, we already arrived at destination. Nothing to do anymore.
            Dashboard.trace(getClass(), "driveDistance() exit");
            return true;
        }
        // ...No, this command is in progress

        // Are we at destination?
        if(isCommandAtDestination(commandId))
        {
            // ...Yes. We need to stop it.
            Dashboard.trace(getClass(), "Command is at destination");
            stop();
            setCommandStatus(commandId, CommandStatus.PROCESSED);
            //code that will run once the robot has arrived
            tickGoal = 0;
            //DON'T TOUCH THE REVERSE CODE. I NEEEEEEEDD IT.
            bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            disComp = true;
            Dashboard.trace(getClass(), "driveDistance() exit");
            return true;
        }
        // No, the robot needs to continue

        writeTrace();
        Dashboard.trace(getClass(), "driveDistance() exit");
        return false;
    }

    public boolean driveDistancePID(String commandId, double distance, double speed, RobotDirection direction) {
        writeTrace();
        double tlspeed, trspeed, blspeed, brspeed;
        // is it a new command?
        if(isCommandNew(commandId))
        {
            // ...Yes, command is new
            // prepare for moving
            Dashboard.trace(getClass(), "New Command. Initializing motors with new target");

            setCommandStatus(commandId, CommandStatus.IN_PROGRESS);
            resetAnglePID();
            resetTickGoal();
            resetSafety();
            tickGoal = convertTicksToInches(distance, direction);
            //tickGoal = 3000;
            if (direction == RobotDirection.FORWARD || direction == RobotDirection.BACKWARD) {
                setTargetTicks(direction, tickGoal);
            } else {
                resetMotorsMode();
            }
            double kp = (direction == RobotDirection.FORWARD || direction == RobotDirection.BACKWARD) ? .08 : 0;
            pidDrive = new PIDController(kp, .0003, 0.001);
            pidDrive.reset();
            pidDrive.setSetpoint(0);
            pidDrive.setInputRange(-90, 90);
            pidDrive.setOutputRange(0, speed);
            pidDrive.enable();

            pidTicks = new PIDController(.05, .003, 0);
            pidTicks.reset();
            pidTicks.setSetpoint(tickGoal);
            pidTicks.setTolerance(10);
            pidTicks.setInputRange(0, tickGoal);
            pidTicks.setOutputRange(0, speed);
            pidTicks.enable();

            Dashboard.trace(getClass(), "driveDistance() exit");
            return false;
        }

        // ...No, this command is not new
        // but is it an old command already processed?
        if(isCommandProcessed(commandId))
        {
            // ...Yes, we already arrived at destination. Nothing to do anymore.
            return true;
        }
        // ...No, this command is in progress

        // Are we at destination?
        if(isCommandAtDestination(commandId) || safetySwitch)
        {
            // ...Yes. We need to stop it.
            Dashboard.trace(getClass(), "Command is at destination");
            stop();
            setCommandStatus(commandId, CommandStatus.PROCESSED);
            //code that will run once the robot has arrived
            tickGoal = 0;
            correction = 0;
            bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            disComp = true;
            Dashboard.trace(getClass(), "driveDistance() exit");
            return true;
        }
        // No, the robot needs to continue
        correction = pidDrive.performPID(getAnglePID());
//        tlspeed = pidTicks.performPID(topLeftMotor.getCurrentPosition());
//        trspeed = pidTicks.performPID(topRightMotor.getCurrentPosition());
//        brspeed = pidTicks.performPID(bottomRightMotor.getCurrentPosition());
//        blspeed = pidTicks.performPID(bottomLeftMotor.getCurrentPosition());
        if (direction == RobotDirection.RIGHT || direction == RobotDirection.LEFT) {
            if (Math.abs(topLeftMotor.getCurrentPosition()) > tickGoal * .5) {
                tlspeed = trspeed = brspeed = blspeed = speed * .3;
            } else if (Math.abs(topLeftMotor.getCurrentPosition()) < tickGoal * .1) {
                tlspeed = trspeed = brspeed = blspeed = speed * .3;
            } else {
                tlspeed = trspeed = brspeed = blspeed = speed;
            }
        } else if (Math.abs(topLeftMotor.getCurrentPosition()) < tickGoal * .2) {
            tlspeed = trspeed = brspeed = blspeed = speed * .3;
        } else {
            tlspeed = trspeed = brspeed = blspeed = speed;
        }
        tlspeed = trspeed = brspeed = blspeed = speed;
        drivePID(direction, correction, tlspeed, trspeed, brspeed, blspeed);
        safetyProtocal();
        return false;
    }

    public void drivePID(RobotDirection direction, double correction, double tlspeed, double trspeed, double brspeed, double blspeed) {
        switch (direction) {
            case FORWARD: drive(tlspeed-correction, trspeed+correction, brspeed+correction, blspeed-correction); break;
            case BACKWARD: drive(-(tlspeed+correction), -(trspeed-correction), -(brspeed-correction), -(blspeed+correction)); break;
            case LEFT: drive(-(tlspeed-correction), trspeed+correction, -(brspeed+correction), blspeed-correction); break;
//            //case LEFT: drive(-(tlspeed), trspeed, -(brspeed), blspeed); break;
            case RIGHT: drive(tlspeed-correction, -(trspeed-correction), brspeed+correction, -(blspeed+correction)); break;
//            //case RIGHT: drive(tlspeed, -(trspeed), brspeed, -(blspeed)); break;
            //case LEFT: drive(.2, 0, 0, 0); break;
        }
    }

    public void setTargetTicks(RobotDirection direction, int pos) {
        switch (direction) {
            case FORWARD: driveToPos(-pos); break;
            case BACKWARD: driveToPos(pos); break;
            case RIGHT: driveToPosSideWays(-pos); break;
            case LEFT: driveToPosSideWays(pos); break;
        }
    }

    public void driveToPosSideWays(int pos)
    {
        Dashboard.trace(getClass(), "driveToPosSideWays() enter");
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //THIS IS A NECESSITY
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        topLeftMotor.setTargetPosition(-pos);
        topRightMotor.setTargetPosition(0);
        bottomLeftMotor.setTargetPosition(0);
        bottomRightMotor.setTargetPosition(0);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dashboard.trace(getClass(), "driveToPosSideWays() exit");
    }

    public boolean selfCorrectAngle(double power) {
        return true;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public boolean rotatePID(int degrees, double power, boolean normalize)
    {
        ElapsedTime timer = new ElapsedTime();
        double delay = normalize ? .3 : 4;
        double normalizeOffset = normalize ? .00007 : 0;
        double lowAngleOffset = Utils.inRange(degrees, -10, 10) ? .04 : 0;
        double lowAngleTolerance = Utils.inRange(degrees, -10, 10) ? 4 : .5;
        // restart imu angle tracking.
        resetAnglePID();
        resetMotorsMode();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        pidRotate = new PIDController(.007+lowAngleOffset, .00003+normalizeOffset, 0);
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(lowAngleTolerance);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAnglePID() == 0 && timer.seconds()<delay)
            {
                topLeftMotor.setPower(power);
                bottomLeftMotor.setPower(power);
                topRightMotor.setPower(-power);
                bottomRightMotor.setPower(-power);
            }

            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be - on right turn.
                topLeftMotor.setPower(-power);
                bottomLeftMotor.setPower(-power);
                topRightMotor.setPower(power);
                bottomRightMotor.setPower(power);
            } while (!pidRotate.onTarget() && timer.seconds()<3);
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAnglePID()); // power will be + on left turn.
                topLeftMotor.setPower(-power);
                bottomLeftMotor.setPower(-power);
                topRightMotor.setPower(power);
                bottomRightMotor.setPower(power);
            } while (!pidRotate.onTarget() && timer.seconds()<3);

        // turn the motors off.
        drive(0);

        rotation = getAnglePID();

        // wait for rotation to stop.

        // reset angle tracking on new heading.
        resetAnglePID();
        return true;
    }

    public boolean setRotation(String commandId, double angle, double speed, RobotDirection direction) {
        if (isCommandNew(commandId)) {
            //start up code
            setCommandStatus(commandId, CommandStatus.IN_PROGRESS);
            tickAngGoal = convertTicksToDegrees(angle, true, direction);
            drivePos_Direction(tickAngGoal, speed, direction);
            return false;
        }
        if (isCommandProcessed(commandId)) {
            //finalization
            return true;
        }

        if (isCommandAtAngle(commandId)) {
            //code that will run once at angle
            setCommandStatus(commandId, CommandStatus.PROCESSED);
            //code that will run once the robot has arrived
            tickAngGoal = 0;
            stop();
            resetMotorsMode();
            resetWheelsMode();
            //DON'T TOUCH THE REVERSE CODE. I NEEEEEEEDD IT.
            bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            disComp = true;
            return true;
        }
        return false;
    }

    public boolean goToAngle(String commandId, double angle, double speed, RobotDirection direction) {
        if (isCommandNew(commandId)) {
            //start up code
            setCommandStatus(commandId, CommandStatus.IN_PROGRESS);
            GyroAngGoal = angle;
            if (direction == RobotDirection.ROTATE_LEFT) {
                driveRotateTentative(speed);
            } else if (direction == RobotDirection.ROTATE_RIGHT) {
                driveRotateTentative(-speed);
            }
            return false;
        }
        if (isCommandProcessed(commandId)) {
            //finalization
            return true;
        }

        if (isCommandAtGyroAngle(commandId, getAngle(), angle)) {
            //code that will run once at angle
            setCommandStatus(commandId, CommandStatus.PROCESSED);
            //code that will run once the robot has arrived
            stop();
            resetMotorsMode();
            resetWheelsMode();
            //DON'T TOUCH THE REVERSE CODE. I NEEEEEEEDD IT.
            bottomLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            topLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            disComp = true;
            return true;
        }
        return false;
    }


    //movement with gamepad for teleop
    public void teleop(Gamepad gamepad) {
        Dashboard.trace(getClass(), "teleop() enter");
        // Left stick controls movement
        double gamepad_left_stick_x = gamepad.left_stick_x * directionScale;
        double gamepad_left_stick_y = -gamepad.left_stick_y * directionScale;

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

        Dashboard.addData("Gamepad Left", Utils.toNearestHundredth(gamepad_left_stick_x) + ", " + Utils.toNearestHundredth(gamepad_left_stick_y));
        Dashboard.addData("Gamepad Right", Utils.toNearestHundredth(gamepad_right_stick_x) + ", " + Utils.toNearestHundredth(gamepad_right_stick_y));
        Dashboard.addData("Movement Scalar", movementScalar);
        Dashboard.addData("Joystick R", r);

        topLeftMotor.setPower(tlPower * movementScalar);
        topRightMotor.setPower(trPower * movementScalar);
        bottomLeftMotor.setPower(blPower * movementScalar);
        bottomRightMotor.setPower(brPower * movementScalar);

        writeTrace();
        Dashboard.trace(getClass(), "teleop() exit");
    }

    public void drive_Direction(double speed, RobotDirection d)
    {
        Dashboard.trace(getClass(), "drive_Direction() enter");
        switch (d) {
            case FORWARD: drive(speed); break;
            case BACKWARD: drive(-speed); break;
            case RIGHT: driveSideways(-speed); break;
            case LEFT: driveSideways(speed); break;
        }
        Dashboard.trace(getClass(), "drive_Direction() exit");
    }

    //positive is right, negative is left
    public void drive_Rotate(double speed) {
        speed = Utils.speedLimiter(speed);
        topLeftMotor.setPower(speed);
        topRightMotor.setPower(-speed);
        bottomLeftMotor.setPower(speed);
        bottomRightMotor.setPower(-speed);
    }

    public void drivePos_Direction(int pos, double speed, RobotDirection d)
    {
        Dashboard.trace(getClass(), "drivePos_Direction() enter");
        switch (d) {
            case FORWARD: driveToPos(pos); break;
            case BACKWARD: driveToPos(-pos); break;
            case RIGHT: driveToPosSideWays(pos); break;
            case LEFT: driveToPosSideWays(-pos); break;
            case ROTATE_LEFT: driveToPosRotateTentative(pos, speed); break;
            case ROTATE_RIGHT: driveToPosRotateTentative(-pos, speed); break;
        }
        Dashboard.trace(getClass(), "drivePos_Direction() exit");
    }

    //WARNING ONE: THIS FUNCTION DOESN'T MAKE SENSE. IT WON'T. WHAT YOU THINK WILL MAKE IT WORK, LIKELY WON'T.
    //WARNING TWO: MAKES MOTORS GO FORWARD, THIS IS A NECESSITY, IF USED MAKE SURE TO SET MOTORS BACK TO REVERSE.
    // POSITIVE POS WILL MAKE IT GO FORWARD. NEGATIVE WILL MAKE IT BACKWARD.
    public void driveToPos(int pos)
    {
        Dashboard.trace(getClass(), "driveToPos() enter");
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //THIS IS A NECESSITY
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        topLeftMotor.setTargetPosition(pos);
        topRightMotor.setTargetPosition(-pos);
        bottomLeftMotor.setTargetPosition(pos);
        bottomRightMotor.setTargetPosition(-pos);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dashboard.trace(getClass(), "driveToPos() exit");
    }

    /** we should use the angle in order to stop
     * Strategy.robotStartAngle
     */
    public void driveToPosRotateTentative(int pos, double speed)
    {
        Dashboard.trace(getClass(), "driveToPosRotateTentative() enter");
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //THIS IS A NECESSITY
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        topLeftMotor.setTargetPosition(pos);
        topRightMotor.setTargetPosition(pos);
        bottomLeftMotor.setTargetPosition(pos);
        bottomRightMotor.setTargetPosition(pos);

        drive(-speed);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dashboard.trace(getClass(), "driveToPos() exit");
    }

    public void driveRotateTentative(double speed) {
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //THIS IS A NECESSITY
        bottomLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        topLeftMotor.setDirection(DcMotor.Direction.FORWARD);

        drive(-speed);


    }

    public void drive(double speed)
    {
        Dashboard.trace(getClass(), "drive(speed) enter");
        speed = Utils.speedLimiter(speed);
        topLeftMotor.setPower(speed);
        topRightMotor.setPower(speed);
        bottomLeftMotor.setPower(speed);
        bottomRightMotor.setPower(speed);
        Dashboard.trace(getClass(), "drive(speed) exit");
    }

    public void stop()
    {
        drive(0);
    }

    public void driveForwardNormal() // Tested to go straight
    {
        drive(.30, .08 , .08, .12);
    }

    public void driveBackwardNormal() // Tested to go straight
    {
        drive(-.18, -.08 , -.08, -.14);
    }

    public void driveSlideLeftNormal() // Tested to go straight
    {
        drive(-.30, +.08 , -.08, +.12); // slide left
    }

    public void driveSlideRightNormal() // Tested to go straight
    {
        drive(+.30, -.08 , +.08, -.08); // slide right
    }

    public void driveRotateLeftNormal() // Tested to go straight
    {
        double gain = 2.;
        drive(-.22*gain, +.15*gain , +.15*gain, -.15*gain); // rotate left
    }

    public void driveRotateRightNormal() // Tested to go straight
    {
        drive(+.25, -.08 , -.08, +.16); // rotate right
    }

    /** negative speeds will go left, positive speeds will go right */
    public void driveSideways(double speed)
    {
        Dashboard.trace(getClass(), "driveSideways() enter");
        speed = Utils.speedLimiter(speed);
        topLeftMotor.setPower(-speed);
        topRightMotor.setPower(speed);
        bottomLeftMotor.setPower(speed);
        bottomRightMotor.setPower(-speed);
        writeTrace();
        Dashboard.trace(getClass(), "driveSideways() exit");
    }

    public void resetWheelsMode()
    {
        Dashboard.trace(getClass(), "resetWheelsMode() enter");
        wheelEnYRIGHT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelEnYLEFT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelEnXBACK.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set modes for motors
        wheelEnYRIGHT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelEnYLEFT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wheelEnXBACK.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Dashboard.trace(getClass(), "resetWheelsMode() exit");
    }

    public void resetMotorsMode()
    {
        Dashboard.trace(getClass(), "resetMotorsMode() enter");
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Dashboard.trace(getClass(), "resetMotorsMode() exit");
    }

    public void setRunningEncoders() {
        topLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    /** for testing wheels */
    public void drive(double topLeftSpeed, double topRightSpeed, double bottomRightSpeed, double bottomLeftSpeed)
    {
        Dashboard.trace(getClass(), "drive() enter");
        topLeftMotor.setPower(Utils.speedLimiter(topLeftSpeed));
        topRightMotor.setPower(Utils.speedLimiter(topRightSpeed));
        bottomRightMotor.setPower(Utils.speedLimiter(bottomRightSpeed));
        bottomLeftMotor.setPower(Utils.speedLimiter(bottomLeftSpeed));
        writeTrace();
        Dashboard.trace(getClass(), "drive() exit");
    }

    public void driveTopLeftMotor(double speed) {
        topLeftMotor.setPower(speed);
    }

    /** Z is the angle we care about */
    public double getAngle()
    {
        return convertAngle(imu.getAngleZ());
    }

    private double getAnglePID()
    {
        double angles = imu.getAngleZ();
        double deltaAngle = angles - lastAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngle = angles;
        return globalAngle;
    }

    private void resetAnglePID()
    {
        lastAngle = imu.getAngleZ();
        globalAngle = 0;
    }


    public void writeTrace()
    {
        if (Strategy.TRACE_ENABLED) {
            Dashboard.addData("TL Motor : " , topLeftMotor.getDirection() + ", " + Utils.toNearestHundredth(topLeftMotor.getPower()));
            Dashboard.addData("TR Motor : " , topRightMotor.getDirection()+ ", " + Utils.toNearestHundredth(topRightMotor.getPower()));
            Dashboard.addData("BR Motor : " , bottomRightMotor.getDirection()+ ", " + Utils.toNearestHundredth(bottomRightMotor.getPower()));
            Dashboard.addData("BL Motor : " , bottomLeftMotor.getDirection()+ ", " + Utils.toNearestHundredth(bottomLeftMotor.getPower()));
            Dashboard.addData("TL Target: " , topLeftMotor.getCurrentPosition()+"/"+ topLeftMotor.getTargetPosition());
            Dashboard.addData("TR Target: " , topRightMotor.getCurrentPosition()+"/"+ topRightMotor.getTargetPosition());
            Dashboard.addData("BR Target: " , bottomRightMotor.getCurrentPosition()+"/"+ bottomRightMotor.getTargetPosition());
            Dashboard.addData("BL Target: " , bottomLeftMotor.getCurrentPosition()+"/"+ bottomLeftMotor.getTargetPosition());
            Dashboard.addData("Tick Goal: ", tickGoal);
            Dashboard.addData("Angle    : ", Utils.toNearestHundredth(getAngle()));

            Dashboard.trace(getClass(),"Angle    : " + Utils.toNearestHundredth(getAngle()));
            Dashboard.trace(getClass(),"Tick Goal: " + tickGoal);
            Dashboard.trace(getClass(),"TL Motor : " + topLeftMotor.getDirection()
                    + ", SPEED=" + Utils.toNearestHundredth(topLeftMotor.getPower())
                    + ", TARGET=" + topLeftMotor.getCurrentPosition()+"/"+ topLeftMotor.getTargetPosition());
            Dashboard.trace(getClass(),"TR Motor : " + topRightMotor.getDirection()
                    + "  SPEED=" + Utils.toNearestHundredth(topRightMotor.getPower())
                    + ", TARGET=" + topRightMotor.getCurrentPosition()+"/"+ topRightMotor.getTargetPosition());
            Dashboard.trace(getClass(),"BR Motor : " + bottomRightMotor.getDirection()
                    + ", SPEED=" + Utils.toNearestHundredth(bottomRightMotor.getPower())
                    + ", TARGET=" + bottomRightMotor.getCurrentPosition()+"/"+ bottomRightMotor.getTargetPosition());
            Dashboard.trace(getClass(),"BL Motor : " + bottomLeftMotor.getDirection()
                    + ", SPEED=" + Utils.toNearestHundredth(bottomLeftMotor.getPower())
                    + ", TARGET=" + bottomLeftMotor.getCurrentPosition()+"/"+ bottomLeftMotor.getTargetPosition());
        }

    }

    //drives drivetrain slower
    public void slowMode() {
        if (!slowed) {
            movementScalar = slowedValue;
            directionScale = -1;
            slowed = true;
        } else {
            movementScalar = 1;
            directionScale = 1;
            slowed = false;
        }
    }
}
