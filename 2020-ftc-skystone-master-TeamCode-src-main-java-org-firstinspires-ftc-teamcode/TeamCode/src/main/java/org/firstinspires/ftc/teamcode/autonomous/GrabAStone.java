package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.Strategy;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Timer;
import java.util.TimerTask;

public class GrabAStone extends Command
{
	boolean isGrabbingSecondStone;

	public GrabAStone(boolean isGrabbingSecondStone)
	{
		this.isGrabbingSecondStone = isGrabbingSecondStone;
		setTimeout(3);
	}

	public GrabAStone()
	{
		this.isGrabbingSecondStone = false;
		setTimeout(3);
	}

	@Override
	public void init() {
		Grabber.getInstance();
		Intake.getInstance().deploy();
		// prepare the instance
	}

	@Override
	public void execute() {
		DriveTrain.getInstance().driveDistance(getID(), 20, .5, DriveTrain.RobotDirection.FORWARD);

		//
		// For cube close to fence, intake is asymetric
		//
		if(isGrabbingSecondStone && (Strategy.position == Strategy.SkystonePosition.THIRD))
		{
			if(Strategy.allianceColor == Strategy.AllianceColor.BLUE) {
				Intake.getInstance().startIntakeLeftInward();
				if (getSeconds() > 1.0)
					Intake.getInstance().startIntakeRightInward();
			}
			else
			{
				Intake.getInstance().startIntakeRightInward();
				if (getSeconds() > 1.0)
					Intake.getInstance().startIntakeLeftInward();
			}
		}
		else
		{
			Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.INWARD);
		}
	}

	@Override
	public boolean isFinished()
	{
		//return Grabber.getInstance().detectStone();
		return true;
	}

	@Override
	public void end() {
		int ticksReached = DriveTrain.getInstance().getTempTicks();
		Strategy.trackedTicks = ticksReached;
		DriveTrain.getInstance().stop();
		Intake.getInstance().activatePushServoAndGrab(300);
		// We can have 2 cubes!!!
		// Make sure we eject the extra cube during 500ms
//		Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.OUTWARD, false);
		new Timer().schedule(new TimerTask() {
			@Override
			public void run() {
				Intake.getInstance().setIntakeDirection(Intake.IntakeDirection.STOPPED);
			}
		}, 500);
	}

}
