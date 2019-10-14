package Team4450.Robot19;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubSystem
{
	private Robot 	robot;
	private boolean	frontClimbExtended = false, rearClimbExtended = false;
	
	// This variable and method make sure this class is a singleton.
	
	private static Climber climber = null;
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Climber getInstance(Robot robot) 
	{
		Util.consoleLog();
		
		if (climber == null) climber = new Climber(robot);
		
		return climber;
	}
	
	// Private constructor prevents multiple instances from being created.
	
	private Climber(Robot robot) 
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		//initialize();
		
		Util.consoleLog("Climber created!");
	}
	
	public void initialize()
	{
		Util.consoleLog();
		
		retractFrontClimb(true);
		
		retractRearClimb();
	}
	
	public void disable()
	{
		Util.consoleLog();
		
		retractFrontClimb(true);
		
		retractRearClimb();
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		Util.consoleLog();
		
		disable();

		climber =  null;
	}
	
	// This is the rest of the class.
	
	public boolean isFrontExtended()
	{
		return frontClimbExtended;
	}
	
	public boolean isRearExtended()
	{
		return rearClimbExtended;
	}
	
	public void extendFrontClimb(boolean override)
	{
		Util.consoleLog();
		
		if (!override && rearClimbExtended) return;
		
		Devices.frontClimbValve.Open();
		
		frontClimbExtended = true;
		
		updateDS();
	}
	
	public void retractFrontClimb(boolean override)
	{
		Util.consoleLog();
		
		if (!override && !rearClimbExtended) return;
		
		Devices.frontClimbValve.Close();
		
		frontClimbExtended = false;
		
		updateDS();
	}
	
	public void extendRearClimb(boolean override)
	{
		Util.consoleLog();
		
		if (!override && !frontClimbExtended) return;
		
		Devices.rearClimbValve.Open();
		
		rearClimbExtended = true;
		
		updateDS();
	}
	
	public void retractRearClimb()
	{
		Util.consoleLog();
		
		Devices.rearClimbValve.Close();
		
		rearClimbExtended = false;
		
		updateDS();
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("FrontClimbExtended", frontClimbExtended);
		SmartDashboard.putBoolean("RearClimbExtended", rearClimbExtended);
	}
}