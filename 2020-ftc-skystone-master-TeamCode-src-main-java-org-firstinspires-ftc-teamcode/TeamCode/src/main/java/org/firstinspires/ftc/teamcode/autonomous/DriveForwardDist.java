package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.statemachine.Command;

public class DriveForwardDist extends Command {
    double distanceInInches;
    double speed;


    public DriveForwardDist(double distanceInInches, double speed)
    {
        this.distanceInInches = distanceInInches;
        this.speed = speed;
        setTimeout(1); // small value for testing, should use encoders
    }

    @Override
    public void init() {
        DriveTrain.getInstance().setdisComp(false);
        DriveTrain.getInstance().resetWheelsMode();
    }

    @Override
    public void execute() {
        DriveTrain.getInstance().driveDistance(getID(), distanceInInches, speed, DriveTrain.RobotDirection.FORWARD);
    }

    // once the inches have been complete
    @Override
    public boolean isFinished()
    {
        return DriveTrain.getInstance().getdisComp();
    }

    @Override
    public void end() {
        DriveTrain.getInstance().stop();
    }

}
