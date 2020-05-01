package org.firstinspires.ftc.teamcode;


public class Strategy {

	public final static boolean TRACE_ENABLED = false;

	// Dimension
	public static final double STONE_WIDTH  = 8.00; // inches
	public static final double STONE_DEPTH  = 4.00; // inches
	public static final double ROBOT_WIDTH  = 4.00; // inches
	public static final double ROBOT_LENGTH = 22.00; // inches
	public static final double STONE_FROM_WALL = 47.00; // inches

	// Alliance color
	public enum AllianceColor { BLUE, RED }
	public static AllianceColor allianceColor;

    // Skystone Position
	public enum SkystonePosition { UNKNOWN, ONE, SECOND, THIRD }
	public static SkystonePosition position;

	// Starting Position
	public enum ROBOT_POSITION {PLATFORM, STONE}
	private static ROBOT_POSITION robotPosition = ROBOT_POSITION.PLATFORM; // have to set a default
	public static double robotStartAngle;

	// Starting Alliance Color
	public enum ROBOT_ALLIANCE {RED, BLUE}
	private static ROBOT_ALLIANCE robotAlliance = ROBOT_ALLIANCE.BLUE; // have to set a default

	public static int trackedTicks = -1;

	public void init()
	{
		// readAlliancePositionAndDetermineStrategy
	}

	public static double getOriginalDistanceFromBuildZone()
	{
		switch(position)
		{
			case ONE   : return 43.0;
			case SECOND: return 43.0+STONE_WIDTH;
			case THIRD : return 43.0+STONE_WIDTH+STONE_WIDTH;
		}
		return 43;
	}

	public static double getDistanceStoneOffset() {
		switch(position)
		{
			case ONE   : return -STONE_WIDTH;
			case SECOND: return 0;
			case THIRD : return STONE_WIDTH;
		}
		return 0;
	}

	public static boolean isRobotPositionPlatform()
	{
		return robotPosition == ROBOT_POSITION.PLATFORM;
	}

	public static boolean isRobotPositionStone()
	{
		return robotPosition == ROBOT_POSITION.STONE;
	}



}




/*
	// Strategy constant for autonomous
	public static enum STRATEGY 
	{
		// LEFT or RIGHT POSITION
			GO_TO_SAME_SIDE_SWITCH,
			GO_TO_SAME_SIDE_SCALE,
		// MIDDLE POSITION
			GO_TO_SWITCH_FROM_MIDDLE,
		// CROSS the line
			CROSS_THE_LINE
	}
	private static STRATEGY strategy;
	
	// Variables 
	private static Alliance alliance;
	
	// FMS Variables
	private static enum SWITCH_POSITION {LEFT, RIGHT, UNDEFINED};
	private static SWITCH_POSITION switchPosition = SWITCH_POSITION.UNDEFINED;
	private static enum SCALE_POSITION {LEFT, RIGHT, UNDEFINED};
	private static SCALE_POSITION scalePosition = SCALE_POSITION.UNDEFINED;
	
	// Starting Position for autonomous
	public static enum ROBOT_POSITION {DEFAULT_FROM_FMS, LEFT, MIDDLE, RIGHT};
	private static ROBOT_POSITION robotPosition = ROBOT_POSITION.DEFAULT_FROM_FMS; // why not!
	
	// End Position for autonomous
	private static enum AUTO_TARGET {SWITCH, SCALE, CROSS, UNDEFINED};
	private static AUTO_TARGET autoTarget = AUTO_TARGET.UNDEFINED;
	
	
	

	public static void readAlliancePositionAndDetermineStrategy()
	{ //readAlliancePositionAndDetermineStrategy
		// Alliance
		alliance = DriverStation.getInstance().getAlliance();
		
		// Position from FMS
		//robot must be in front of driver station
		switch(DriverStation.getInstance().getLocation())
		{
			case 1: 	robotPosition = ROBOT_POSITION.LEFT; break;
			case 2: 	robotPosition = ROBOT_POSITION.MIDDLE; break;
			case 3: 	robotPosition = ROBOT_POSITION.RIGHT; break;
		}
		
		// switch position from FMS
		if(DriverStation.getInstance().getGameSpecificMessage().length() > 0) 
		{
			switch(DriverStation.getInstance().getGameSpecificMessage().charAt(0))
			{
				case 'L':	switchPosition = SWITCH_POSITION.LEFT; break;
				case 'R':	switchPosition = SWITCH_POSITION.RIGHT; break;
				default: 		break;
			}
			
			// scale position from FMS
			switch(DriverStation.getInstance().getGameSpecificMessage().charAt(1))
			{
				case 'L':	scalePosition = SCALE_POSITION.LEFT; break;
				case 'R': 	scalePosition = SCALE_POSITION.RIGHT; break;
				default:	break;
			}
		}
		
		switch(Robot.positionChooser.getSelected())	
		{
			case "LEFT": 		
				robotPosition = ROBOT_POSITION.LEFT; 
				break;
			case "MIDDLE": 		
				robotPosition = ROBOT_POSITION.MIDDLE; 
				break;
			case "RIGHT": 		
				robotPosition = ROBOT_POSITION.RIGHT; 
				break;
			default:			
				break;
		}
		
		
//		robotPosition = ROBOT_POSITION.LEFT;
//		robotPosition = ROBOT_POSITION.MIDDLE;
//		robotPosition = ROBOT_POSITION.RIGHT;

		

		// Which Strategy?
		switch(robotPosition)
		{
			case LEFT:
			case RIGHT:	
					if (isSwitchPlateAndRobotOnTheSameSide())
						// no question here, we go to our switch 
						strategy = STRATEGY.GO_TO_SAME_SIDE_SWITCH;
					else if (isScalePlateAndRobotOnTheSameSide())
						// scale is on our side, we want to own it
						strategy = STRATEGY.GO_TO_SAME_SIDE_SCALE;
					else 
						// both switch and scale are on the opposite side
						// we are crossing the line
						strategy = STRATEGY.CROSS_THE_LINE;
					break;
			case MIDDLE: 
					if (switchPosition == SWITCH_POSITION.UNDEFINED)
						// don't know where our switch is, should not happen, but you never know
						strategy = STRATEGY.CROSS_THE_LINE;
					else
						// go to switch from middle position
						strategy = STRATEGY.GO_TO_SWITCH_FROM_MIDDLE;
					break;
			default:	// Robot position is unknown, should not happen, but you never know
					strategy = STRATEGY.CROSS_THE_LINE;
					break;
		}
		
		
		Console.log("\n### ====> Alliance: " + alliance + ", Position: " +robotPosition + ", Strategy: " +strategy);
	}
	
	
	public static boolean isSwitchPlateAndRobotOnTheSameSide()
	{
		return (robotPosition == ROBOT_POSITION.LEFT && switchPosition == SWITCH_POSITION.LEFT)
			|| (robotPosition == ROBOT_POSITION.RIGHT && switchPosition == SWITCH_POSITION.RIGHT);
	}
	
	public static boolean isScalePlateAndRobotOnTheSameSide()
	{
		return (robotPosition == ROBOT_POSITION.LEFT && scalePosition == SCALE_POSITION.LEFT)
			|| (robotPosition == ROBOT_POSITION.RIGHT && scalePosition == SCALE_POSITION.RIGHT);
	}
	
	public static boolean weAreNotSureTheOtherSideRobotWillDeliverACubeInSwitch()
	{
		// get the value from smart dashboard
		return false;
	}
	
	public static boolean isRobotPositionLeft()
	{
		return robotPosition == ROBOT_POSITION.LEFT;
	}
	
	public static boolean isRobotPositionRight()
	{
		return robotPosition == ROBOT_POSITION.RIGHT;
	}
	
	public static boolean isRobotPositionMiddle()
	{
		return robotPosition == ROBOT_POSITION.MIDDLE;
	}
	
	public static boolean isSwitchPlateLeft()
	{
		return switchPosition == SWITCH_POSITION.LEFT;
	}
	
	public static boolean isSwitchPlateRight()
	{
		return switchPosition == SWITCH_POSITION.RIGHT;
	}
	
	public static STRATEGY getStrategy()
	{
		return strategy;
	}
	
	public static boolean isAllianceBlue()
	{
		return DriverStation.getInstance().getAlliance() == Alliance.Blue;
	}
	
	public static boolean isAllianceRed()
	{
		return DriverStation.getInstance().getAlliance() == Alliance.Red;
	}
	
	public static String isSwitchL()
	{
		return DriverStation.getInstance().getGameSpecificMessage();
	}
	
	public static boolean isDemoMode()
	{
		return false; //strategy == STRATEGY.DEMO_MODE;
	}
}
*/