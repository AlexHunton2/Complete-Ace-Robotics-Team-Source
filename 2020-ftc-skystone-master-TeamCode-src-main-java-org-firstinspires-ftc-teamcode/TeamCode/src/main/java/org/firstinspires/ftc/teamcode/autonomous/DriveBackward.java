package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveBackward extends Command
{
	double distanceInInches;
	double speed;
	boolean isFinished = false;


	public DriveBackward(double distanceInInches, double speed)
	{
		this.distanceInInches = distanceInInches;
		this.speed = speed;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {

	}

	@Override
	public void execute()
	{
		isFinished = DriveTrain.getInstance().driveDistance(getID(), distanceInInches, speed, DriveTrain.RobotDirection.BACKWARD);
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
