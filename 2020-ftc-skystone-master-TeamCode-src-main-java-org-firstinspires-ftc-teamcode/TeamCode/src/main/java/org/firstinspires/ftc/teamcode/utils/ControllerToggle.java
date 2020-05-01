package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Grabber;

public class ControllerToggle {
    private Gamepad gamepad;
    private Grabber grabber;

    private boolean buttonADown;
    private boolean buttonBDown;
    private boolean buttonYDown;
    private boolean buttonXDown;
    private boolean leftBumperDown;
    private boolean rightBumperDown;
    private boolean DPadLeftDown;
    private boolean DPadDownDown;
    private boolean DPadUpDown;
    private boolean colorSensorDown;
    private boolean colorSensorUp;
    private boolean DPadRightDown;
    private boolean backDown;
    private boolean startDown;

    private boolean previousButtonAState;
    private boolean previousButtonBState;
    private boolean previousButtonYState;
    private boolean previousLeftBumperState;
    private boolean previousRightBumperState;
    private boolean previousButtonXState;
    private boolean previousDPadLeftState;
    private boolean previousDPadDownState;
    private boolean previousDPadUpState;
    private boolean previousColorSensorState;
    private boolean previousDPadRightState;
    private boolean previousBackDown;
    private boolean previousStartDown;

    public ControllerToggle(Gamepad _gamepad) {
        gamepad = _gamepad;
        grabber = Grabber.getInstance();
    }

    //called at beginning of update loop in AceTeleOpMode.java
    public void updateButtonStates() {
        // on down
        if (gamepad.a == true && previousButtonAState == false)
            buttonADown = true;
        if (gamepad.b == true && previousButtonBState == false)
            buttonBDown = true;
        if (gamepad.y == true && previousButtonYState == false)
            buttonYDown = true;
        if (gamepad.x == true && previousButtonXState == false)
            buttonXDown = true;
        if (gamepad.left_bumper == true && previousLeftBumperState == false)
            leftBumperDown = true;
        if (gamepad.right_bumper == true && previousRightBumperState == false)
            rightBumperDown = true;
        if (gamepad.dpad_left == true && previousDPadLeftState == false)
            DPadLeftDown = true;
        if (gamepad.dpad_down == true && previousDPadDownState == false)
            DPadDownDown = true;
        if (gamepad.dpad_up == true && previousDPadUpState == false)
            DPadUpDown = true;
        if (gamepad.dpad_right == true && previousDPadRightState == false)
            DPadRightDown = true;
        if (gamepad.back == true && previousBackDown == false)
            backDown = true;
        if (gamepad.start == true && previousStartDown == false)
            startDown = true;


        //on up
//        if (grabber.detectStone() == false && previousColorSensorState == true)
//           colorSensorUp = true;


        previousButtonAState = gamepad.a;
        previousButtonBState = gamepad.b;
        previousButtonYState = gamepad.y;
        previousButtonXState = gamepad.x;
        previousLeftBumperState = gamepad.left_bumper;
        previousRightBumperState = gamepad.right_bumper;
        previousDPadLeftState = gamepad.dpad_left;
        previousDPadDownState = gamepad.dpad_down;
        previousDPadUpState = gamepad.dpad_up;
        previousDPadRightState = gamepad.dpad_right;
        previousBackDown = gamepad.back;
        previousStartDown = gamepad.start;
    }

    //down accessors
    public boolean getADown() {
        boolean bool = buttonADown;
        buttonADown = false;
        return bool;
    }

    public boolean getBDown() {
        boolean bool = buttonBDown;
        buttonBDown = false;
        return bool;
    }

    public boolean getXDown() {
        boolean bool = buttonXDown;
        buttonXDown = false;
        return bool;
    }

    public boolean getYDown() {
        boolean bool = buttonYDown;
        buttonYDown = false;
        return bool;
    }

    public boolean getLBDown() {
        boolean bool = leftBumperDown;
        leftBumperDown = false;
        return bool;
    }

    public boolean getRBDown() {
        boolean bool = rightBumperDown;
        rightBumperDown = false;
        return bool;
    }

    public boolean getDPadLeftDown() {
        boolean bool = DPadLeftDown;
        DPadLeftDown = false;
        return bool;
    }

    public boolean getDPadDownDown() {
        boolean bool = DPadDownDown;
        DPadDownDown = false;
        return bool;
    }

    public boolean getDPadUpDown() {
        boolean bool = DPadUpDown;
        DPadUpDown = false;
        return bool;
    }

    public boolean getDPadRightDown() {
        boolean bool = DPadRightDown;
        DPadRightDown = false;
        return bool;
    }

    public boolean getBackDown() {
        boolean bool = backDown;
        backDown = false;
        return bool;
    }

    public boolean getStartDown() {
        boolean bool = startDown;
        startDown = false;
        return bool;
    }
}
