package Team4450.Robot19;

import Team4450.Lib.Util;

public class Lift 
{
	private Robot robot;
	
	// This variable used to make this class is a singleton.
	
	public static Lift lift = null;
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Lift getInstance(Robot robot) 
	{
		if (lift == null) lift = new Lift(robot);
		
		return lift;
	}
	
	// Private constructor prevents multiple instances from being created.
	
	private Lift(Robot robot) 
	{
		this.robot = robot;
		
		Util.consoleLog("Lift created!");
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		lift =  null;
	}
	
	// This is the rest of the class.
}
