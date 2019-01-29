package Team4450.Robot19;

import org.opencv.core.Mat;

import Team4450.Lib.Util;

public class Vision 
{
	private Robot robot;
	
	// This variable and method make sure this class is a singleton.
	
	public static Vision vision = null;
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Vision getInstance(Robot robot) 
	{
		if (vision == null) vision = new Vision(robot);
		
		return vision;
	}
	
	// Private constructor prevents multiple instances from being created.
	
	private Vision(Robot robot) 
	{
		this.robot = robot;
		
		Util.consoleLog("Vision created!");
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		vision =  null;
	}
	
	// This is the rest of the class.
}
