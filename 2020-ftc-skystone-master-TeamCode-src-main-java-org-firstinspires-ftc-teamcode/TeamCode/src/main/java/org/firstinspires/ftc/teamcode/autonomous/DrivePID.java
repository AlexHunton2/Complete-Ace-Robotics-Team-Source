package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DrivePID extends Command
{
	double distanceInInches;
	double speed;
	DriveTrain.RobotDirection direction;
	boolean isFinished = false;


	public DrivePID(double distanceInInches, double speed, DriveTrain.RobotDirection direction)
	{
		this.distanceInInches = distanceInInches;
		this.speed = speed;
		this.direction = direction;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {
	}

	@Override
	public void execute() {
		isFinished = DriveTrain.getInstance().driveDistancePID(getID(), distanceInInches, speed, direction);
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
