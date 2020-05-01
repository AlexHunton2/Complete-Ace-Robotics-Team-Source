package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class EjectAStone extends Command
{

	public EjectAStone()
	{
		setTimeout(1.2	); // eject should be quick
	}

	@Override
	public void init()
	{
		// I want to open the claw here
		// Grabber.getInstance().open()
		Grabber.getInstance().setGrabbed(false);
		Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.OUTWARD);
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
		Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.STOPPED); // would rather have a stop() method
		Grabber.getInstance().resetInstance();
		Intake.getInstance().stop();
	}

}
