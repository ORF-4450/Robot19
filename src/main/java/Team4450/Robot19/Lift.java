package Team4450.Robot19;

import Team4450.Lib.Util;
import Team4450.Robot19.Devices;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift 
{
	private Robot robot;
	private boolean				holdingPosition = false, holdingHeight = false, holdingHatchHeight = false;
	private boolean				hatchReleased = false, hatchMidPosition = false;
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
		
		// Set hatch grabber to hold hatch position.
		grabHatch();
				
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
		SmartDashboard.putBoolean("LiftHoldingHeight", holdingHeight);
		SmartDashboard.putBoolean("LiftHoldingPosition", holdingPosition);
		SmartDashboard.putBoolean("HatchHoldingHeight", holdingHatchHeight);
		SmartDashboard.putBoolean("HatchReleased", hatchReleased);
	}
	
	// Set lift winch motors to arbitrary power with top and bottom detection.
	
	public void setWinchPower(double power)
	{
		if (isHoldingHeight()) return;
		
		// Limit power going down. May limit going up as well.
		if (power < 0) 
			power = -.10;
		else
			power = power * 1.0;
		
		if (Devices.winchEncoderEnabled)
		{
			// limit switch and hall effect sensor read in reverse so two sets of code.
			
			// limit switch form.
//			if ((power > 0 && Devices.winchEncoder.get() < 10800) ||	
//				(power < 0 && !Devices.winchSwitch.get()))
//				Devices.winchDrive.set(power);
//			else
//			{
//				if (Devices.winchSwitch.get()) Devices.winchEncoder.reset();
//				
//				Devices.winchDrive.set(0);
//			}
			
			// hall effect sensor form.
			if ((power > 0 && Devices.winchEncoder.get() < 1500) ||	// 10800
				(power < 0 && Devices.winchSwitch.get()))
				Devices.winchDrive.set(power);
			else
			{
				if (!Devices.winchSwitch.get()) Devices.winchEncoder.reset();
				
				Devices.winchDrive.set(0);
			}
		}
		else
			Devices.winchDrive.set(power);
	}
	
	// 380 cargo ship  749 rocket lv2  1349 rocket lv3
	// Automatically move lift to specified encoder count and hold it there.
	// count < 0 turns pid controller off.
	
	public void setHeight(int count)
	{
		Util.consoleLog("%d", count);
		
		if (count >= 0)
		{
			if (isHoldingPosition()) holdPosition(0);
			
			// p,i,d values are a guess. p is based on 750 range * .0002 = .15 max power.
			// i,d values start at 0 and are adjusted to reduce overshoot and speed to 
			// reach steady state.
			// f value is the base motor power to apply to move to encoder target count.
			// Setpoint is the target encoder count.
			// The idea is that the difference between the current encoder count and the
			// target count will apply power to bring the two counts together and stay there.
			liftPidController.setPID(0.0002, 0.00005, 0.0003, 0.0);
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
			liftPidController.setPID(0.0002, 0.00005, 0.0003, speed);
			liftPidController.setSetpoint(Devices.winchEncoder.get());
			liftPidController.setOutputRange(-1, 1);
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
	
	// Set hatch winch motors to arbitrary power.
	
	public void setHatchPower(double power)
	{
		if (isHoldingHatchHeight()) return;
		
		// Reduce power going down.
		//if (power < 0) power = power * .50;
		
		power = power * .25;
		
		Devices.hatchWinch.set(power);
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
			hatchPidController.setPID(0.00003, 0.00001, 0.0003, 0.0);
			//liftPidController.setPID(0.0003, 0.0, 0.0, 0.0);
			hatchPidController.setOutputRange(-1, 1);
			hatchPidController.setSetpoint(count);
			hatchPidController.setPercentTolerance(1);	// % error.
			hatchPidController.enable();
			holdingHatchHeight = true;
		}
		else
		{
			hatchPidController.disable();
			holdingHatchHeight = false;
		}
		
		updateDS();
	}
	
	public boolean isHoldingHatchHeight()
	{
		return holdingHatchHeight;
	}
	
	public void setHatchHeightAuto()
	{
		if (!hatchMidPosition)
		{
			hatchMidPosition = true;
			setHatchHeight(20000);
		}
		else
		{
			hatchMidPosition = false;
			setHatchHeight(41000);
		}
	}
	
	public void releaseHatch()
	{
		Util.consoleLog();
		
		Devices.hatchReleaseValve.Close();
		
		hatchReleased = true;
		
		updateDS();
	}
	
	public void grabHatch()
	{
		Util.consoleLog();
		
		Devices.hatchReleaseValve.Open();
		
		hatchReleased = false;
		
		updateDS();
	}

	public boolean isHatchReleased()
	{
		return hatchReleased;
	}
}
