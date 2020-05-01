package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.statemachine.CommandListener;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class Test_Parallel_EjectAStone extends Command
{

	public Test_Parallel_EjectAStone()
	{
		super();
		setTimeout(1.0);
	}

	@Override
	public void init()
	{
	}

	@Override
	public void execute()
	{
		// I want to eject the stone
        // Intake.getInstance().eject()
		Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.OUTWARD);

	}

	@Override
	public boolean isFinished()
	{
		// timeout defined in the constructor will be automatic
		// always return false here
		return false;
	}

	@Override
	public void end()
	{
		Intake.getInstance().stop();
	}

}
