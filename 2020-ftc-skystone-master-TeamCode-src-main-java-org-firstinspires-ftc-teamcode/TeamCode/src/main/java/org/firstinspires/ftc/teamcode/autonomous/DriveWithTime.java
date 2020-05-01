package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

public class DriveWithTime extends Command
{
	double speed;
	DriveTrain.RobotDirection direction;
	boolean isFinished = false;


	public DriveWithTime(double speed, DriveTrain.RobotDirection direction, double time)
	{
		this.direction = direction;
		this.speed = speed;
		setTimeout(time); // small value for testing, should use encoders
	}

	@Override
	public void init() {
		//Intake.getInstance().deploy(); Why deploy the intake here?
	}

	@Override
	public void execute() {
		DriveTrain.getInstance().drivePID(direction, 0, speed, speed, speed, speed);
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
