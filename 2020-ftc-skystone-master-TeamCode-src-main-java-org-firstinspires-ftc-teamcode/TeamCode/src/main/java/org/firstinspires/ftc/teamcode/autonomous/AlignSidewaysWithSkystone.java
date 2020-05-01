package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class AlignSidewaysWithSkystone extends Command
{
	boolean isFinished = false;
	double speed;
	double distance = -1.0;
	DriveTrain.RobotDirection direction = null;

	public AlignSidewaysWithSkystone(double speed)
	{
		this.speed = speed;
	}

	public AlignSidewaysWithSkystone(double distance, DriveTrain.RobotDirection direction, double speed)
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
		if (Strategy.allianceColor==Strategy.AllianceColor.BLUE) {
			switch (Strategy.position) {
				case ONE:
					isFinished = DriveTrain.getInstance().driveDistancePID(getID(), Strategy.STONE_WIDTH-3, speed, DriveTrain.RobotDirection.BACKWARD);
					break;
				case SECOND: /* nothing to do, we are already aligned */
					isFinished = true;
					break;
				case THIRD:
					isFinished = DriveTrain.getInstance().driveDistancePID(getID(), Strategy.STONE_WIDTH+3, speed, DriveTrain.RobotDirection.FORWARD);
					break;
			}
		}
		else if (Strategy.allianceColor==Strategy.AllianceColor.RED) {
			switch (Strategy.position) {
				case ONE:
					isFinished = DriveTrain.getInstance().driveDistancePID(getID(), Strategy.STONE_WIDTH-3, speed, DriveTrain.RobotDirection.FORWARD);
					break;
				case SECOND: /* nothing to do, we are already aligned */
					isFinished = DriveTrain.getInstance().driveDistancePID(getID(), Strategy.STONE_WIDTH, speed, DriveTrain.RobotDirection.BACKWARD);
					isFinished = true;
					break;
				case THIRD:
					isFinished = DriveTrain.getInstance().driveDistancePID(getID(), Strategy.STONE_WIDTH+3, speed, DriveTrain.RobotDirection.BACKWARD);
					break;
			}
		}


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
