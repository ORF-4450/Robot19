package Team4450.Robot19;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import Team4450.Robot19.Devices;

public class Pickup 
{
	private Robot 	robot;
	private boolean	extended = false, intakeRunning = false, spitRunning = false, autoIntake = false;
	private Thread	autoIntakeThread;
	
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
		
		retract();
		
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
	
	public void extend()
	{
		Util.consoleLog();
		
		Devices.pickupValve.SetA();
		
		extended = true;
		
		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();

		Devices.pickupValve.SetB();
		
		extended = false;
		
		updateDS();
	}
	
	public boolean isExtended()
	{
		return extended;
	}
	
	public void intakeBall()
	{
		Util.consoleLog("running=%b", intakeRunning);
		
		if (intakeRunning)
		{
			Devices.ballSpit.set(-.05);		// Create a braking effect to hold ball.
			Devices.pickupMotor.stopMotor();
			intakeRunning = false;
			retract();
		}
		else
		{
			extend();
			Devices.ballSpit.set(.20);
			Devices.pickupMotor.set(.70);
			intakeRunning = true;
		}
		
		updateDS();
	}
	
	public boolean isIntakeRunning()
	{
		return intakeRunning;
	}
	
	public void spitBall()
	{
		Util.consoleLog();
		
		if (isAutoIntakeRunning()) stopAutoIntake();
		
		if (intakeRunning) intakeBall();
		
		spitRunning = true;
		
		Devices.ballSpit.set(.60);
		Devices.pickupMotor.set(-.60);

		Timer.delay(.5);
		
		Devices.ballSpit.stopMotor();
		Devices.pickupMotor.stopMotor();
		
		spitRunning = false;
		
		updateDS();
	}
	
	public boolean isSpitRunning()
	{
		return spitRunning;
	}
	
	public boolean isAutoIntakeRunning()
	{
		return autoIntake;
	}
	
	/**
	 * Start auto cube Intake thread.
	 */
	public void startAutoIntake()
	{
		Util.consoleLog();
		
		if (autoIntakeThread != null) return;

		autoIntakeThread = new AutoIntake();
		autoIntakeThread.start();
	}
	
	/**
	 * Stops auto Intake thread.
	 */
	public void stopAutoIntake()
	{
		Util.consoleLog();

		if (autoIntakeThread != null) autoIntakeThread.interrupt();
		
		autoIntakeThread = null;
	}

	//----------------------------------------
	// Automatic cube Intake thread.
	
	private class AutoIntake extends Thread
	{
		AutoIntake()
		{
			Util.consoleLog();
			
			this.setName("AutoBallIntake");
	    }
		
	    public void run()
	    {
	    	boolean ballDetected = false;
	    	
	    	Util.consoleLog();
	    	
	    	try
	    	{
	    		autoIntake = true;
	    		
	    		updateDS();
	    		
	    		intakeBall();

	    		sleep(250);
	    		
    	    	while (!isInterrupted() && robot.isEnabled())
    	    	{
    	    		// while looping we watch for ball limit switch detection (false). 
    	    		// When it goes true again we stop.
    	    		
    	    		if (!Devices.ballSwitch.get()) ballDetected = true;
    	    		
    	    		if  (ballDetected) Devices.ballSpit.set(.20);
    	    		
    	    		if (ballDetected && Devices.ballSwitch.get()) break;
    	    		
    	            // We sleep since JS updates come from DS every 20ms or so. We wait 50ms so this thread
    	            // does not run at the same time as the teleop thread.
    	            sleep(30);
    	    	}
    	    	
    	    	if (!interrupted() && robot.isEnabled()) Util.consoleLog("  ball loaded");
	    	}
	    	catch (InterruptedException e) { intakeBall(); }
	    	catch (Throwable e) { e.printStackTrace(Util.logPrintStream); }
	    	finally { if (intakeRunning) intakeBall(); }
			
	    	autoIntake = false;
			autoIntakeThread = null;
			
			updateDS();
	    }
	}	// end of AutoIntake thread class.

	private void updateDS()
	{
		SmartDashboard.putBoolean("Intake", intakeRunning);
		SmartDashboard.putBoolean("Spit", spitRunning);
		SmartDashboard.putBoolean("AutoIntake", autoIntake);
		SmartDashboard.putBoolean("IntakeExtended", extended);
	}
}