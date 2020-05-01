package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class TapeMeasure extends Command
{
	boolean isFinished = false;
	private Grabber grabber = Grabber.getInstance();
	private Intake.tapeActions action;



	public TapeMeasure(Intake.tapeActions action)
	{
		setTimeout(1.5); // small value for testing, should use encoders
		this.action = action;
	}

	@Override
	public void init() {
		Intake.getInstance().setTapeMeasureServo(action);
	}

	@Override
	public void execute() {
		//No loops
	}

	@Override
	public boolean isFinished()
	{
		return isFinished;
	}

	@Override
	public void end() {

	}

}
