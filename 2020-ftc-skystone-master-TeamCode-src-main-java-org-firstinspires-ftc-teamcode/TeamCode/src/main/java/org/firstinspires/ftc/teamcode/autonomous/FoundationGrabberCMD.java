package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.FoundationGrabber;

public class FoundationGrabberCMD extends Command {

    boolean isFinished = false;
    boolean Toggle;
    FoundationGrabber foundationGrabber;

    public FoundationGrabberCMD(boolean toggle)
    {
        setTimeout(10);
        this.Toggle = toggle;
    }

    @Override
    public void init() {
        foundationGrabber = FoundationGrabber.getInstance();
    }

    @Override
    public void execute() {
        foundationGrabber.setFoundationGrabbed(Toggle);
        isFinished = true;
    }

    @Override
    public boolean isFinished()
    {
        return isFinished;
    }

    @Override
    public void end() {
        DriveTrain.getInstance().stop();
    }
}
