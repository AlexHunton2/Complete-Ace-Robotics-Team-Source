package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.KeonAceTeleopMode;
import org.firstinspires.ftc.teamcode.statemachine.Command;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Grabber;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.Timer;
import java.util.TimerTask;

public class DriveBackwardAndGimbleUp extends Command
{
	double distanceInInches;
	double speed;
	boolean isFinishedWithGrabber=false,isFinishedDriving = false;


	public DriveBackwardAndGimbleUp(double distanceInInches, double speed)
	{
		this.distanceInInches = distanceInInches;
		this.speed = speed;
		setTimeout(10); // small value for testing, should use encoders
	}

	@Override
	public void init() {
		Intake.getInstance().deploy();
		Grabber.getInstance().setGrabbed(true);
	}

	@Override
	public void execute()
	{

		isFinishedDriving = DriveTrain.getInstance().driveDistance(getID(), distanceInInches, speed, DriveTrain.RobotDirection.BACKWARD);
	}

	@Override
	public boolean isFinished()
	{
		return isFinishedWithGrabber && isFinishedDriving;
	}

	@Override
	public void end() {
		DriveTrain.getInstance().stop();
	}
}
