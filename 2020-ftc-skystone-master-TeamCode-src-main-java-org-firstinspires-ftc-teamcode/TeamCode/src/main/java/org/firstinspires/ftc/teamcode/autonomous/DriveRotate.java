package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveRotate extends Command
{
	private DriveTrain.RobotDirection direction = DriveTrain.RobotDirection.ROTATE_LEFT;
	double speed;
	double angle;
	boolean isFinished = false;


	public DriveRotate(DriveTrain.RobotDirection direction, double speed, double angle)
	{
		this.direction = direction;
		this.speed = speed;
		this.angle = angle;
		setTimeout(10);
	}

	@Override
	public void init() {

	}

	@Override
	public void execute() {
//		DriveTrain.getInstance().driveRotateLeftNormal();
		isFinished = DriveTrain.getInstance().setRotation(getID(), angle, speed, direction);
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
