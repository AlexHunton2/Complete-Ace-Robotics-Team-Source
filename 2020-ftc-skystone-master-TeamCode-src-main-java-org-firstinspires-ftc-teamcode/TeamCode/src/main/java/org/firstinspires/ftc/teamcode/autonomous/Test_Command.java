package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;

public class Test_Command extends Command
{
	String label;

	public Test_Command(String label, double maxseconds)
	{
		super(maxseconds);
		this.label = label;
	}

	@Override
	public void init()
	{

	}

	@Override
	public void execute()
	{
		// does nothing
	}

	@Override
	public boolean isFinished()
	{
		 return false;
	}

	@Override
	public void end()
	{

	}

	@Override
	public String toString()
	{
		return label;
	}
}
