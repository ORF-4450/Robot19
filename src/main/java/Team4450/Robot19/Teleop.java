
package Team4450.Robot19;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Robot19.Devices;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop
{
	private final Robot 		robot;
	public  JoyStick			rightStick, leftStick, utilityStick;
	public  LaunchPad			launchPad;
	private boolean				autoTarget, altDriveMode;
	private Vision				vision;
	private GearBox				gearBox;
	private Lift				lift;
	private Pickup				pickup;
	private Climber				climber;
	
	// This variable used to make this class is a singleton.
	
	public static Teleop teleop = null;
	
	// Private constructor prevents multiple instances from being created.

	private Teleop(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		//Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;

		gearBox = GearBox.getInstance(robot);
		
		vision = Vision.getInstance(robot);
		
		lift = Lift.getInstance(robot);
		
		pickup = Pickup.getInstance(robot);
		
		climber = Climber.getInstance(robot);
	}
	
	/**
	* Get reference to the single instance of this class shared by any caller of
	* this method.
	* @return Reference to single shared instance of this class.
	*/
	public static Teleop getInstance(Robot robot) 
	{
		if (teleop == null) teleop = new Teleop(robot);
		
		return teleop;
	}
	
	/**
	* Release any resources allocated and the singleton object.
	*/
	void dispose()
	{
		Util.consoleLog();

		if (leftStick != null) leftStick.dispose();
		if (rightStick != null) rightStick.dispose();
		if (utilityStick != null) utilityStick.dispose();
		if (launchPad != null) launchPad.dispose();
		if (gearBox != null) gearBox.dispose();
		if (lift != null) lift.dispose();
		if (pickup != null) pickup.dispose();
		if (climber != null) climber.dispose();
		
		teleop = null;
	}

	void OperatorControl() throws Exception
	{
		double	rightY = 0, leftY = 0, utilX = 0, utilY = 0, rightX = 0, leftX = 0;
		double	gain = .05;
		boolean	steeringAssistMode = false;
		int		angle;

		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();

		LCD.printLine(1, "Mode: teleop All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());
		
		// Set synchronousPID as a sendable for testing.
		//SynchronousPID pidTest = new SynchronousPID(1,2,3,4);
		//pidTest.setSetpoint(99);		
		//SmartDashboard.putData(pidTest);

		// Configure LaunchPad and Joystick event handlers.

		launchPad = new LaunchPad(Devices.launchPad, this);
//		launchPad = new LaunchPad(Devices.launchPad, LaunchPadControlIDs.BUTTON_RED, this);
//
//		LaunchPadControl lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_BACK);
//		lpControl.controlType = LaunchPadControlTypes.SWITCH;
//
//		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_LEFT_FRONT);
//		lpControl.controlType = LaunchPadControlTypes.SWITCH;
//
//		lpControl = launchPad.AddControl(LaunchPadControlIDs.ROCKER_RIGHT);
//		lpControl.controlType = LaunchPadControlTypes.SWITCH;
//
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_GREEN);
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_YELLOW);
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE);
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLUE_RIGHT);
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_BLACK);
//		launchPad.AddControl(LaunchPadControlIDs.BUTTON_RED_RIGHT);
		launchPad.addLaunchPadEventListener(new LaunchPadListener());
//		launchPad.Start();

		leftStick = new JoyStick(Devices.leftStick, "LeftStick", JoyStickButtonIDs.TRIGGER, this);
		//Example on how to track button:
		leftStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		leftStick.addJoyStickEventListener(new LeftStickListener());
		leftStick.Start();

		rightStick = new JoyStick(Devices.rightStick, "RightStick", JoyStickButtonIDs.TRIGGER, this);
		//Example on how to track button:
		rightStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
		rightStick.addJoyStickEventListener(new RightStickListener());
		rightStick.Start();
		
		// Invert for h drive correct direction.
		//rightStick.invertX(true);

		utilityStick = new JoyStick(Devices.utilityStick, "UtilityStick", this);
//		utilityStick = new JoyStick(Devices.utilityStick, "UtilityStick", JoyStickButtonIDs.TRIGGER, this);
//		//Example on how to track button:
//		utilityStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
//		utilityStick.AddButton(JoyStickButtonIDs.TOP_LEFT);
//		utilityStick.AddButton(JoyStickButtonIDs.TOP_RIGHT);
//		utilityStick.AddButton(JoyStickButtonIDs.TOP_BACK);
		utilityStick.addJoyStickEventListener(new UtilityStickListener());
//		utilityStick.Start();

		// Invert driving joy sticks Y axis so + values mean forward.
		leftStick.invertY(true);
		rightStick.invertY(true);
		
		utilityStick.deadZoneY(.20);

		// Set CAN Talon brake mode by rocker switch setting.
		// We do this here so that the Utility stick thread has time to read the initial state
		// of the rocker switch. Depends on lpcontrol being the last control defined for the 
		// launch pad as the one that controls brake mode.
		//if (robot.isComp) Devices.SetCANTalonBrakeMode(lpControl.latchedState);
		
		// 2018 post season testing showed Anakin liked this setting, smoothing driving.
		Devices.SetCANTalonRampRate(0.5);
		
		// Set Navx current yaw to 0.
		Devices.navx.resetYaw();

		// Reset wheel encoders.
		Devices.leftEncoder.reset();
		Devices.rightEncoder.reset();
		Devices.leftEncoder.resetMaxRate();
		Devices.rightEncoder.resetMaxRate();
		
		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);

		// Driving loop runs until teleop is over.

		Util.consoleLog("enter driving loop");
		boolean firsttime = true;
		
		while (robot.isEnabled())	// && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			//rightY = stickLogCorrection(rightStick.GetY());	// fwd/back
			//leftY = stickLogCorrection(leftStick.GetY());	// fwd/back

			//rightX = stickLogCorrection(rightStick.GetX());	// left/right
			//leftX = stickLogCorrection(leftStick.GetX());	// left/right
			
			rightY = rightStick.GetY();	// fwd/back
			leftY = leftStick.GetY();	// fwd/back

			rightX = rightStick.GetX();	// left/right
			leftX = leftStick.GetX();	// left/right

			utilY = utilityStick.GetY();

			LCD.printLine(2, "leftenc=%d  rightenc=%d - wEnc=%d  hEnc=%d", Devices.leftEncoder.get(), Devices.rightEncoder.get(), 
					Devices.winchEncoder.get(), Devices.hatchEncoder.get());			
			LCD.printLine(3, "leftY=%.3f (%.3f)  rightY=(%.3f) %.3f (%.3f)  rightX=%.3f  utilY=%.3f", leftY, 
					 Devices.LRCanTalon.get(), rightStick.GetY(), rightY, Devices.RRCanTalon.get(), rightX, utilY);
			LCD.printLine(4, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), 
					Devices.navx.getTotalYaw(), Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(5, "wEnc=%d  hEnc=%d", Devices.winchEncoder.get(), Devices.hatchEncoder.get());
			LCD.printLine(6, "wSwitch=%b  ballsw=%b  ballsensor=%d", Devices.winchSwitch.get(), Devices.ballSwitch.get(),
					Devices.ballSensor.getValue());
			LCD.printLine(10, "pressureV=%.2f  psi=%d  ustb=%b", robot.monitorCompressorThread.getVoltage(), 
					robot.monitorCompressorThread.getPressure(), utilityStick.GetCurrentState(JoyStickButtonIDs.TOP_BACK));
			
			// set H drive motors. Apply some proportional power to drive wheels to counter the
			// H drive tendency to drive in an arc.
			
			if (!autoTarget && rightStick.GetCurrentState(JoyStickButtonIDs.TRIGGER))
			{
				Devices.hDrive.set(rightX * .50);
				
				if (rightX < 0)
					leftY = Math.abs(rightX) * .50;
				else
					rightY = rightX * .50;
			}
			else
				Devices.hDrive.set(0);
			
			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			// Two drive modes, full tank and alternate. Switch on right stick trigger (not used 2019).

			if (!autoTarget) 
			{
				if (altDriveMode)
				{	// normal tank with straight drive assist when sticks within 10% of each other and
					// right stick power is greater than 50%.
					if (isLeftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50)
					{
						// Reset angle measurement when entering this code first time after mode is enabled.
						if (!steeringAssistMode) Devices.navx.resetYaw();

						// Angle is negative if robot veering left, positive if veering right when going forward.
						// It is opposite when going backward.

						angle = (int) Devices.navx.getYaw();

						LCD.printLine(5, "angle=%d", angle);

						// Invert angle for backwards movement.

						if (rightY < 0) angle = -angle;

						//Util.consoleLog("angle=%d", angle);

						// Note we invert sign on the angle because we want the robot to turn in the opposite
						// direction than it is currently going to correct it. So a + angle says robot is veering
						// right so we set the turn value to - because - is a turn left which corrects our right
						// drift.
						
						Devices.robotDrive.curvatureDrive(rightY, -angle * gain, false);

						steeringAssistMode = true;
					}
					else
					{
						steeringAssistMode = false;
						Devices.robotDrive.tankDrive(leftY, rightY);	// Normal tank drive.
					}

					SmartDashboard.putBoolean("SteeringAssist", steeringAssistMode);
				}
				else
					Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
				
					// This shows how to use curvature drive mode, toggled by trigger (for testing).
					//Devices.robotDrive.curvatureDrive(rightY, rightX, rightStick.GetLatchedState(JoyStickButtonIDs.TRIGGER));
			}

			if (firsttime) Util.consoleLog("after tank drive");
			
			firsttime = false;
			
			// Set lift winch/hatch winch power.

			if (utilityStick.GetCurrentState(JoyStickButtonIDs.TOP_BACK))
			{
				//if (lift.isHoldingHeight()) lift.setHeight(-1);
				
				lift.setHatchPower(utilY);	//squareInput(utilY));
			}
			else
			{
				lift.setHatchPower(0);
				lift.setWinchPower(Util.squareInput(utilY));
			}
			
			// Update the robot heading indicator on the DS. Only for labview DB.

			//SmartDashboard.putNumber("Gyro", Devices.navx.getHeadingInt());
			
			// Cause smartdashboard to update any registered Sendables, including Gyro2.
			
			SmartDashboard.updateValues();

			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		// ensure we start next time in low gear.
		gearBox.lowSpeed();
		
		Util.consoleLog("end");
	}

//	// This method scales the input values less than 1.0 down but on a sliding scale
//	// so that 1.0 input returns 1.0 output. This is what is in the DifferentialDrive.tankDrive
//	// method that scales input to reduce sensitivity.
//	
//	private double squareInput(double input)
//	{
//		return Math.copySign(input * input, input);	
//	}
	
	private boolean isLeftRightEqual(double left, double right, double percent)
	{
		if (Math.abs(left - right) <= (1 * (percent / 100))) return true;

		return false;
	}

	// Custom base logarithm.
	// Returns logarithm base of the value.

	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0 to 1.0 to the motor working power range of approx 0.5 to 1.0 using
	// logarithmic curve.

	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);

		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);

		return joystickValue;
	}

	// Handle LaunchPad control events.

	public class LaunchPadListener implements LaunchPadEventListener 
	{
		public void ButtonDown(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s, latchedState=%b ord=%d hc=%d %s", control.id.name(),  control.latchedState, control.id.ordinal(), control.id.hashCode(), control.id.getClass().toString());

			switch(control.id)
			{
				case BUTTON_GREEN:
//					if (gearBox.isLowSpeed())
//		    			gearBox.highSpeed();
//		    		else
//		    			gearBox.lowSpeed();
		
					if (control.latchedState)
						lift.setHatchHeight(-100);
					else
						lift.setHatchHeight(-200);
					
					break;
					
				case BUTTON_RED:
					Devices.leftEncoder.reset();
					Devices.rightEncoder.reset();
					break;
					
				case BUTTON_BLUE:
					Util.consoleLog("blue");
					if (climber.isFrontExtended())
						climber.retractFrontClimb(true);	// Remove overrides after testing.
					else
						climber.extendFrontClimb(true);
					
					break;
					
				case BUTTON_BLACK:
					if (climber.isRearExtended())
						climber.retractRearClimb();
					else
						climber.extendRearClimb(true);
					
					break;
					
				case BUTTON_BLUE_RIGHT:
					//lift.setHeight(1000);
					if (pickup.isExtended())
						pickup.retract();
					else
						pickup.extend();
					
					break;
					
				case BUTTON_RED_RIGHT:
					if (lift.isHoldingHeight())
						lift.setHeight(-1);
					else
						lift.setHeight(1250);
					
					break;
					
				case BUTTON_YELLOW:
					if (lift.isHoldingHeight())
						lift.setHeight(-1);
					else
						lift.setHeight(750);
					
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(LaunchPadEvent launchPadEvent) 
		{
			//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
		}

		public void SwitchChange(LaunchPadEvent launchPadEvent) 
		{
			LaunchPadControl	control = launchPadEvent.control;

			Util.consoleLog("%s", control.id.name());

			switch(control.id)
			{
				// Set CAN Talon brake mode.
	    		case ROCKER_LEFT_BACK:
    				if (Devices.isBrakeMode())
    					Devices.SetCANTalonBrakeMode(false);	// coast
    				else
    					Devices.SetCANTalonBrakeMode(true);		// brake
    				
    				break;
    				
	    		case ROCKER_LEFT_FRONT:
					if (robot.cameraThread != null) robot.cameraThread.ChangeCamera();
	    			break;
	    			
				default:
					break;
			}
		}
	}

	// Handle Right JoyStick Button events.

	private class RightStickListener implements JoyStickEventListener 
	{

		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				// Trigger used in teleop loop to enable H drive.
//				case TRIGGER:
//					altDriveMode = !altDriveMode;
//					break;

			//Example of Joystick Button case:
			/*
			case BUTTON_NAME_HERE:
				if (button.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
				
				case TOP_MIDDLE:
					if (lift.isHoldingHeight())
						lift.setHeight(-1);
					else
						lift.setHeight(350);
					
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Left JoyStick Button events.

	private class LeftStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					if (gearBox.isLowSpeed())
	    				gearBox.highSpeed();
	    			else
	    				gearBox.lowSpeed();

					break;
					
				case TOP_BACK:
					altDriveMode = !altDriveMode;
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.name());
		}
	}

	// Handle Utility JoyStick Button events.

	private class UtilityStickListener implements JoyStickEventListener 
	{
		public void ButtonDown(JoyStickEvent joyStickEvent) 
		{
			JoyStickButton	button = joyStickEvent.button;

			Util.consoleLog("%s, latchedState=%b", button.id.name(),  button.latchedState);

			switch(button.id)
			{
				case TRIGGER:
					if (lift.isHatchReleased())
						lift.resetHatch();
					else
						lift.releaseHatch();
					
					break;
					
				case TOP_LEFT:
					pickup.intakeBall();
					break;
					
				case TOP_RIGHT:
					pickup.spitBall();
					break;
					
				case TOP_MIDDLE:
					if (pickup.isAutoIntakeRunning())
						pickup.stopAutoIntake();
					else
						pickup.startAutoIntake();
					
					break;
					
				default:
					break;
			}
		}

		public void ButtonUp(JoyStickEvent joyStickEvent) 
		{
			//Util.consoleLog("%s", joyStickEvent.button.id.name());
		}
	}
}
