package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;

import java.util.Timer;
import java.util.TimerTask;

public class NormalizeElevator extends Command
{
	boolean isFinished = false;


	public NormalizeElevator()
	{

		setTimeout(1); // small value for testing, should use encoders
	}

	@Override
	public void init() {
		Elevator.getInstance().setLevel(69);
	}

	@Override
	public void execute()
	{

		new Timer().schedule(
				new TimerTask() {
					@Override
					public void run() {
						Elevator.getInstance().setLevel(0);
						Elevator.getInstance().resetAngleMotor();
						isFinished = true;
					}
				},
				500
		);
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
