package Team4450.Robot19;

import Team4450.Lib.Util;
import Team4450.Robot19.Devices;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift 
{
	private Robot robot;
	private boolean				holdingPosition, holdingHeight;
	private final PIDController	liftPidController, hatchPidController;

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
		
		liftPidController = new PIDController(0.0, 0.0, 0.0, Devices.winchEncoder, Devices.winchDrive);
		
		hatchPidController = new PIDController(0.0, 0.0, 0.0, Devices.hatchEncoder, Devices.hatchWinch);
		
		Devices.winchEncoder.reset();
		Devices.hatchEncoder.reset();
				
		updateDS();
		
		Util.consoleLog("Lift created!");
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	public void dispose()
	{
		Util.consoleLog();
		
		liftPidController.disable();
		liftPidController.close();
		
		hatchPidController.disable();
		hatchPidController.close();

		lift =  null;
	}
	
	// This is the rest of the class.
	
	public void updateDS()
	{
		SmartDashboard.putBoolean("TargetLocked", holdingHeight);
	}
	
	// Set lift winch motors to arbitrary power with top and bottom detection.
	
	public void setWinchPower(double power)
	{
		if (isHoldingHeight()) return;
		
		if (Devices.winchEncoderEnabled)
		{
			// limit switch and hall effect sensor read in reverse so two sets of code.
			
			// limit switch form.
			if ((power > 0 && Devices.winchEncoder.get() < 10800) ||	
				(power < 0 && !Devices.winchSwitch.get()))
				Devices.winchDrive.set(power);
			else
			{
				if (Devices.winchSwitch.get()) Devices.winchEncoder.reset();
				
				Devices.winchDrive.set(0);
			}
			
//				// hall effect sensor form.
//				if ((power > 0 && Devices.winchEncoder.get() < 14000) ||	// 10800
//					(power < 0 && Devices.winchSwitch.get()))
//					Devices.winchDrive.set(power);
//				else
//				{
//					if (!Devices.winchSwitch.get()) Devices.winchEncoder.reset();
//					
//					Devices.winchDrive.set(0);
//				}
		}
		else
			Devices.winchDrive.set(power);
	}
	
	// Automatically move lift to specified encoder count and hold it there.
	// count < 0 turns pid controller off.
	
	public void setHeight(int count)
	{
		Util.consoleLog("%d", count);
		
		if (count >= 0)
		{
			if (isHoldingPosition()) holdPosition(0);
			
			// p,i,d values are a guess.
			// f value is the motor power to apply to move to encoder target count.
			// Setpoint is the target encoder count.
			// The idea is that the difference between the current encoder count and the
			// target count will apply power to bring the two counts together and stay there.
			liftPidController.setPID(0.0003, 0.00001, 0.0003, 0.0);
			//liftPidController.setPID(0.0003, 0.0, 0.0, 0.0);
			liftPidController.setOutputRange(-1, 1);
			liftPidController.setSetpoint(count);
			liftPidController.setPercentTolerance(1);	// % error.
			liftPidController.enable();
			holdingHeight = true;
		}
		else
		{
			liftPidController.disable();
			holdingHeight = false;
		}
		
		updateDS();
	}
	
	public boolean isHoldingHeight()
	{
		return holdingHeight;
	}
	
	public boolean isHoldingPosition()
	{
		return holdingPosition;
	}
	
	// Automatically hold lift position at specified power level. zero disables.
	
	void holdPosition(double speed)
	{
		Util.consoleLog("%f", speed);
		
		if (speed != 0)
		{
			if (isHoldingHeight()) setHeight(-1);
			
			// p,i,d values are a guess.
			// f value is the base motor speed, which is where (power) we want to hold position.
			// Setpoint is current encoder count.
			// The idea is that any encoder motion will alter motor base speed to hold position.
			liftPidController.setPID(0.0003, 0.00001, 0.0003, speed);
			liftPidController.setSetpoint(Devices.winchEncoder.get());
			liftPidController.setPercentTolerance(1);	// % error.
			liftPidController.enable();
			holdingPosition = true;
		}
		else
		{
			liftPidController.disable();
			holdingPosition = false;
		}
	}
	
	// Automatically move hatch holder to specified encoder count and hold it there.
	// count < 0 turns pid controller off.
	
	public void setHatchHeight(int count)
	{
		Util.consoleLog("%d", count);
		
		if (count >= 0)
		{
			// p,i,d values are a guess.
			// f value is the motor power to apply to move to encoder target count.
			// Setpoint is the target encoder count.
			// The idea is that the difference between the current encoder count and the
			// target count will apply power to bring the two counts together and stay there.
			hatchPidController.setPID(0.0003, 0.00001, 0.0003, 0.0);
			//liftPidController.setPID(0.0003, 0.0, 0.0, 0.0);
			hatchPidController.setOutputRange(-1, 1);
			hatchPidController.setSetpoint(count);
			hatchPidController.setPercentTolerance(1);	// % error.
			hatchPidController.enable();
		}
		else
		{
			hatchPidController.disable();
		}
	}
}
