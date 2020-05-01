package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveBackwardTrack extends Command
{
	double additionalInInches;
	double speed;
	boolean isFinished = false;


	public DriveBackwardTrack(double additionalInInches, double speed)
	{
		this.additionalInInches = additionalInInches;
		this.speed = speed;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {

	}

	@Override
	public void execute()
	{
		double grabInch = DriveTrain.getInstance().convertForTickToInches(Math.abs(Strategy.trackedTicks));
		isFinished = DriveTrain.getInstance().driveDistance(getID(), additionalInInches+grabInch, speed, DriveTrain.RobotDirection.BACKWARD);
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
