package Team4450.Robot19;

import Team4450.Lib.Util;

public class Pickup 
{
	private Robot robot;
	
	// This variable and method make sure this class is a singleton.
	
	public static Pickup pickup = null;
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Pickup getInstance(Robot robot) 
	{
		if (pickup == null) pickup = new Pickup(robot);
		
		return pickup;
	}
	
	// Private constructor prevents multiple instances from being created.
	
	private Pickup(Robot robot) 
	{
		this.robot = robot;
		
		Util.consoleLog("Pickup created!");
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		Util.consoleLog();
		
		pickup =  null;
	}
	
	// This is the rest of the class.
}