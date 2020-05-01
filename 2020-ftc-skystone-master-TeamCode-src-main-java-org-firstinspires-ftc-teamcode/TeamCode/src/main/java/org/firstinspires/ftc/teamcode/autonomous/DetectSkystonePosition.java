package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.utils.Dashboard;
import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.utils.vision.SkystoneDetector;

import java.util.Timer;
import java.util.TimerTask;

public class DetectSkystonePosition extends Command
{
	private Strategy.SkystonePosition position;
	private SkystoneDetector skystoneDetector;


	public DetectSkystonePosition(SkystoneDetector skystoneDetector)
	{
		this.skystoneDetector = skystoneDetector;
	}

	@Override
	public void init()
	{
		// setTelemetry position to unknown
		position = Strategy.SkystonePosition.UNKNOWN;
		Strategy.position = position;
		// start the vision system, it is running in the background
//		skystoneDetector.enable();
	}

	@Override
	public void execute()
	{
		if(Strategy.allianceColor == Strategy.AllianceColor.BLUE)
		{
			switch(skystoneDetector.getSkystonePosition())
			{
				case 1  : position = Strategy.SkystonePosition.ONE; break;
				case 2  : position = Strategy.SkystonePosition.SECOND; break;
				case 3  : position = Strategy.SkystonePosition.THIRD; break;
				default : position = Strategy.SkystonePosition.UNKNOWN; break;
			}
		}
		else if(Strategy.allianceColor == Strategy.AllianceColor.RED)
		{
			switch(skystoneDetector.getSkystonePosition())
			{
				case 1  : position = Strategy.SkystonePosition.THIRD; break;
				case 2  : position = Strategy.SkystonePosition.SECOND; break;
				case 3  : position = Strategy.SkystonePosition.ONE; break;
				default : position = Strategy.SkystonePosition.UNKNOWN; break;
			}
		}

		Dashboard.debug("Position is " + position.toString());
	}

	@Override
	public boolean isFinished()
	{
		Dashboard.addData("getSeconds() = " , getSeconds());
		return (position != Strategy.SkystonePosition.UNKNOWN);
	}

	@Override
	public void end()
	{

		// Skystone not found after Timeout
		if(position == Strategy.SkystonePosition.UNKNOWN)
		{
			// pick a random position
			position = Strategy.SkystonePosition.THIRD;
			Dashboard.debug("Force position to " + position.toString());
		}

		// save the position
		Strategy.position = position;

		// stop the vision system, it is running in the background
		new Timer().schedule(
				new TimerTask() {
					@Override
					public void run() {
						skystoneDetector.disable();
					}
				}, 10
		);

	}


	// ====================================================


	@Override
	public String toString()
	{
		return "Position="+position.toString();
	}
}
