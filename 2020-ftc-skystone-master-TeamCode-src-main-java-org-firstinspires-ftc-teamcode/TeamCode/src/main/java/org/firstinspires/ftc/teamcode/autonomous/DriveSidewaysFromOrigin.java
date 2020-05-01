package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveSidewaysFromOrigin extends Command
{
	boolean isFinished = false;
	double speed;
	double additionalDistance = -1.0;
	DriveTrain.RobotDirection direction = null;

	public DriveSidewaysFromOrigin(double additionalDistance, DriveTrain.RobotDirection direction, double speed)
	{
		this.speed = speed;
		this.additionalDistance = additionalDistance;
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
		isFinished = DriveTrain.getInstance().driveDistance(getID(), Strategy.getOriginalDistanceFromBuildZone()+additionalDistance, speed, direction);
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
