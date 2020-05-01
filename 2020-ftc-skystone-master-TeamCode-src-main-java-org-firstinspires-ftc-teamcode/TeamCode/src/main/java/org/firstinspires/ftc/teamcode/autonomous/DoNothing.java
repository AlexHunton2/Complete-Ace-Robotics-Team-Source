package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class DoNothing extends Command
{
	boolean isFinished = false;
	double speed;
	double distance = -1.0;
	DriveTrain.RobotDirection direction;

	public DoNothing(double timeoutInSeconds)
	{
		setTimeout(timeoutInSeconds);
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
		// Do nothing
	}

	@Override
	public boolean isFinished()
	{
		return false; /* relying on timeout */
	}

	@Override
	public void end() {
		Intake.getInstance();
		DriveTrain.getInstance().stop();
	}

}
