package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DrivePIDRotate extends Command
{
	int angle;
	double speed;
	boolean isFinished = false;


	public DrivePIDRotate(int angle, double speed)
	{
		this.angle = angle;
		this.speed = speed;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {
	}

	@Override
	public void execute() {
		isFinished = DriveTrain.getInstance().rotatePID(angle, speed, false);
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
