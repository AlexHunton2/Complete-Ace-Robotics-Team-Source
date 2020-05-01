package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveSideways extends Command
{
	boolean isFinished = false;
	double speed;
	double distance = -1.0;
	DriveTrain.RobotDirection direction;

	public DriveSideways(double distance, DriveTrain.RobotDirection direction, double speed)
	{
		this.speed = speed;
		this.distance = distance;
		this.direction = direction;
	}

	@Override
	public void init()
	{
		//
		// ROBOT MUST BE POSITIONED IN FRONT OF 2nd STONE
		//
	}

	@Override
	public void execute()
	{
		isFinished = DriveTrain.getInstance().driveDistance(getID(), distance, speed, direction);
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
