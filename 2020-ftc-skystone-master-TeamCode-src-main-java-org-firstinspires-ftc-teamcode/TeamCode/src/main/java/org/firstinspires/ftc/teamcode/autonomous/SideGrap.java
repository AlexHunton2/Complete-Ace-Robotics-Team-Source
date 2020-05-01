package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SideGrap extends Command
{
	boolean isFinished = false;
	private Grabber grabber = Grabber.getInstance();
	private Grabber.SideGripState state;



	public SideGrap(Grabber.SideGripState state)
	{
		setTimeout(1.5); // small value for testing, should use encoders
		this.state = state;
	}

	@Override
	public void init() {
		grabber.setSideGripped(state);
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
