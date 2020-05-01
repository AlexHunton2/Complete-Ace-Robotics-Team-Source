package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.utils.ControllerToggle;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp
public class KeonAceTeleopMode extends AbstractRobotMode {

    //servos go from 1 to 0
    public static float SERVO_MAX = 1;
    public static float SERVO_MIN = 0;
    public static float RED_SERVO_MAX_DEGREES = 270;


    //when this is true, user input will not be accepted.
    public static boolean isAutomated;

    DriveTrain driveTrain;
    Intake intake;
    Elevator elevator;
    Grabber grabber;
    FoundationGrabber foundationGrabber;

    ControllerToggle gamepadControllerToggle;
    ControllerToggle gamepadDriverToggle;

    //debug variables
    int buttonDownCount = 0;

    //gamepadA = Driver
    //gamepadB = Controller

    @Override
    public void init() {
        super.init();
        elevator.resetInstance();
        driveTrain = DriveTrain.getInstance();
        intake = Intake.getInstance();
        elevator = Elevator.getInstance();
        grabber = Grabber.getInstance();
        foundationGrabber = FoundationGrabber.getInstance();


        //only used for the controller gamepad (gamepadB)
        gamepadControllerToggle = new ControllerToggle(gamepadB);
        gamepadDriverToggle = new ControllerToggle(gamepadA);
    }

    @Override
    public void loop() {
        //keep this at beginning of loop
        driveTrain.teleop(gamepadA);
        gamepadControllerToggle.updateButtonStates();
        gamepadDriverToggle.updateButtonStates();

        executeDriverBinds();


        //checks if automated process is happening
        if (!isAutomated)
            executeControllerBinds();

        // move to level
        elevator.moveToLevel(elevator.getLevel());

        //General telemetry data
        Dashboard.addData("TriggerCount", buttonDownCount);
        Dashboard.addData("GamepadB X Down", gamepadB.x);
        Dashboard.addData("GamepadB B Down", gamepadB.b);
        Dashboard.addData("GamepadB A Down", gamepadB.a);
        Dashboard.addData("GamepadB Y Down", gamepadB.y);

        Dashboard.addData("Level", Elevator.getInstance().getLevel());
        Dashboard.addData("Elevator Mode", Elevator.getInstance().getMode());

        Dashboard.addData("GamepadA X Down", gamepadA.x);
        Dashboard.addData("GamepadA B Down", gamepadA.b);
        Dashboard.addData("GamepadA A Down", gamepadA.a);
        Dashboard.addData("GamepadA Y Down", gamepadA.y);


        // Keep any code past this comment at the end of the loop
        telemetry.update();
    }

    void executeDriverBinds() {
        if (gamepadDriverToggle.getLBDown()) {
            driveTrain.slowMode();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getRBDown()) {
            intake.deploy();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getXDown()) {
            grabber.toggleRelease();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getBDown()) {
            grabber.toggleClamp();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getADown()) {
            intake.toggleIntakeInward();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getYDown()) {
            intake.toggleIntakeOutward();
            buttonDownCount++;
        }

        if (gamepadDriverToggle.getStartDown()) {
            buttonDownCount++;
            Intake.getInstance().toggleTapMeasure(Intake.tapeActions.OUT);
        }

        if (gamepadDriverToggle.getBackDown()) {
            buttonDownCount++;
            Intake.getInstance().toggleTapMeasure(Intake.tapeActions.IN);
        }

    }

    void executeControllerBinds() {

        if (gamepadControllerToggle.getBackDown()) {
            foundationGrabber.toggleActive();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getXDown()) {
            //Put into the unbonked position
            grabber.setSpunWithElevator(false);
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getYDown()) {
            elevator.getInstance().incrementLevel();
            intake.getInstance().deploy();
            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    elevator.getInstance().decrementLevel();
                }
            }, 1000);
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getADown()) {
            //manual full bonk (Drop a level, turn intake on, grab)
            manualBonk();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getDPadLeftDown()) {
            grabber.setSpun(false);
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getDPadRightDown()) {
            grabber.setSpun(true);
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getDPadDownDown()) {
            //FULL DOWN.
            startPosition();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getDPadUpDown()) {
            grabber.setSpunWithElevator(true);
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getBDown()) {
            grabber.toggleGrab();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getRBDown()) {
            elevator.incrementLevel();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getLBDown()) {
            elevator.decrementLevel();
            buttonDownCount--;
        }

        if (gamepadControllerToggle.getDPadRightDown()) {
            Elevator.getInstance().bonk();
            buttonDownCount++;
        }

        if (gamepadControllerToggle.getStartDown()) {
            elevator.toggleCapstone();
            buttonDownCount++;
        }
    }

    public static double redServoDegree(double degrees) {
        //clamp from 0 to 270
        degrees = degrees > RED_SERVO_MAX_DEGREES ? RED_SERVO_MAX_DEGREES : degrees;
        return degrees / RED_SERVO_MAX_DEGREES;
    }

    public void manualBonk() {
        //Must be in unbonk position
        if (elevator.getLevel() == 1) {
            grabber.setGrabbed(false);
            grabber.setSpun(false);
            elevator.setLevel(0);
            new Timer().schedule(new TimerTask() {
                @Override
                public void run() {
                    intake.setIntakeDirection(Intake.IntakeDirection.INWARD);
                    new Timer().schedule(new TimerTask() {
                        @Override
                        public void run() {
                            intake.setIntakeDirection(Intake.IntakeDirection.STOPPED);
                            grabber.setGrabbed(true);
                        }
                    }, 1000);
                }
            }, 1000);
        }
    }

    public void startPosition() {
        grabber.setGrabbed(false);
        grabber.setSpun(false);
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                elevator.setLevel(0);
            }
        }, 1000);
    }

    public void stop()
    {
        Dashboard.trace(getClass(), "stop() enter");
        DriveTrain.resetInstance();
        Elevator.resetInstance();
        FoundationGrabber.resetInstance();
        Grabber.resetInstance();
        Intake.resetInstance();
        Dashboard.trace(getClass(), "stop() exit");
    }
}
