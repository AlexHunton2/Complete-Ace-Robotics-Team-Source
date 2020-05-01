package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class NormalizeAnglePID extends Command
{
	int angle;
	double speed;
	boolean isFinished = false;


	public NormalizeAnglePID(double speed)
	{
		this.angle = 0;
		this.speed = speed;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {
	}

	@Override
	public void execute() {
		double rawAngle = DriveTrain.getInstance().getRawAngle();
		if (!Utils.inRange(rawAngle, -4, 4)) {
			this.angle = (int)rawAngle;
			isFinished = DriveTrain.getInstance().rotatePID(-angle, 1, true);

		} else {
			isFinished = true;
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
