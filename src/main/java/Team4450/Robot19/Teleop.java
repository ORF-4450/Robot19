
package Team4450.Robot19;

import java.lang.Math;

import org.opencv.core.Rect;

import Team4450.Lib.*;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Lib.NavX.NavXEvent;
import Team4450.Lib.NavX.NavXEventListener;
import Team4450.Lib.NavX.NavXEventType;
import Team4450.Robot19.Devices;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop
{
	private final Robot 		robot;
	private boolean				autoTarget, altDriveMode;
	private double				timeMarker = 0;

	// This variable used to make this class is a singleton.
	
	private static Teleop 		teleop = null;
	
	// Private constructor prevents multiple instances from being created.

	private Teleop(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		//Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;
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

		Devices.launchPad.removeAllLaunchPadEventListeners();
		Devices.leftStick.removeAllJoyStickEventListeners();
		Devices.rightStick.removeAllJoyStickEventListeners();
		Devices.utilityStick.removeAllJoyStickEventListeners();

		teleop = null;
	}

	void OperatorControl() throws Exception
	{
		double	rightY = 0, leftY = 0, utilX = 0, utilY = 0, rightX = 0, leftX = 0;
		double	gain = .05;
		boolean	steeringAssistMode = false;
		int		angle;

		// Motor safety turned off during initialization.

		Util.consoleLog();

		LCD.printLine(1, "Mode: teleop All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());
		
		// Set synchronousPID as a sendable for testing.
		//SynchronousPID pidTest = new SynchronousPID(1,2,3,4);
		//pidTest.setSetpoint(99);		
		//SmartDashboard.putData(pidTest);

		// Configure LaunchPad and Joystick event handlers.

		Devices.launchPad.addLaunchPadEventListener(new LaunchPadListener());

		Devices.leftStick.addJoyStickEventListener(new LeftStickListener());

		Devices.rightStick.addJoyStickEventListener(new RightStickListener());
		
		Devices.utilityStick.addJoyStickEventListener(new UtilityStickListener());
		
		// Configure NavX collision detection.
		
		Devices.navx.setNavXEventListener(new NavXListener());
		Devices.navx.setEventMonitoringInterval(.050);
   		Devices.navx.setCollisionThreshold(1.0);
   		Devices.navx.enableEventMonitoring(true);
   		
		// Invert driving joy sticks Y axis so + values mean forward.
		Devices.leftStick.invertY(true);
		Devices.rightStick.invertY(true);
		
		Devices.utilityStick.deadZoneY(.20);

		// 2018 post season testing showed Anakin liked this setting, smoothing driving.
		Devices.SetCANTalonRampRate(0.5);
		
		// Set Navx current yaw to 0.
		Devices.navx.resetYaw();

		// Reset wheel encoders.
		Devices.leftEncoder.reset();
		Devices.rightEncoder.reset();
		Devices.leftEncoder.resetMaxRate();
		Devices.rightEncoder.resetMaxRate();
		
		Devices.gearBox.lowSpeed();
		Devices.climber.initialize();
		Devices.pickup.initialize();
		Devices.lift.initialize();
		
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
			
			rightY = Devices.rightStick.GetY();	// fwd/back
			leftY = Devices.leftStick.GetY();	// fwd/back

			rightX = Devices.rightStick.GetX();	// left/right
			leftX = Devices.leftStick.GetX();	// left/right

			utilY = Devices.utilityStick.GetY();

			LCD.printLine(2, "leftenc=%d  rightenc=%d - wEnc=%d  hEnc=%d", Devices.leftEncoder.get(), Devices.rightEncoder.get(), 
					Devices.winchEncoder.get(), Devices.hatchEncoder.get());			
			LCD.printLine(3, "leftY=%.3f (%.3f)  rightY=(%.3f) %.3f (%.3f)  rightX=%.3f  utilY=%.3f", leftY, 
					 Devices.LRCanTalon.get(), Devices.rightStick.GetY(), rightY, Devices.RRCanTalon.get(), rightX, utilY);
			LCD.printLine(4, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), 
					Devices.navx.getTotalYaw(), Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(5, "wEnc=%d  hEnc=%d", Devices.winchEncoder.get(), Devices.hatchEncoder.get());
			LCD.printLine(6, "wSwitch=%b  ballsw=%b  ballsensor=%d", Devices.winchSwitch.get(), Devices.ballSwitch.get(),
					Devices.ballSensor.getValue());
			LCD.printLine(10, "pressureV=%.2f  psi=%d  ustb=%b", robot.monitorCompressorThread.getVoltage(), 
					robot.monitorCompressorThread.getPressure(), Devices.utilityStick.GetCurrentState(JoyStickButtonIDs.TOP_BACK));
			
			// set H drive motors. Apply some proportional power to drive wheels to counter the
			// H drive tendency to drive in an arc.
			
			if (!autoTarget && Devices.rightStick.GetCurrentState(JoyStickButtonIDs.TRIGGER))
			{
				Devices.hDrive.set(rightX * .90);
				
				if (rightX < 0)
					leftY = .35;	//Math.abs(rightX) * .30;
				else
					rightY = .35;	//rightX * .30;
			}
			else
				Devices.hDrive.set(0);
			
			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			// Two drive modes, full tank and alternate. Switch on right stick top back.

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

			if (firsttime) Util.consoleLog("after first loop");
			
			firsttime = false;
			
			// Set lift winch/hatch winch power.

//			if (utilityStick.GetCurrentState(JoyStickButtonIDs.TOP_BACK))
//			{
//				//if (lift.isHoldingHeight()) lift.setHeight(-1);
//				
//				lift.setHatchPower(utilY);	//squareInput(utilY));
//			}
//			else
//			{
//				lift.setHatchPower(0);

			Devices.lift.setWinchPower(Util.squareInput(utilY));

//			}
			
			// Update the robot heading indicator on the DS. Only for labview DB.

			//SmartDashboard.putNumber("Gyro", Devices.navx.getHeadingInt());
			
			// Cause smartdashboard to update any registered Sendables, including Gyro2.
			
			SmartDashboard.updateValues();
			
			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		Util.consoleLog("end");
	}

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
						Devices.lift.setHatchHeight(-100);
					else
						Devices.lift.setHatchHeight(-200);
					
					break;
					
				case BUTTON_RED:
					Devices.leftEncoder.reset();
					Devices.rightEncoder.reset();
					break;
					
				case BUTTON_BLUE:
					Util.consoleLog("blue");
					if (Devices.climber.isFrontExtended())
						Devices.climber.retractFrontClimb(true);	// Remove overrides after testing.
					else
						Devices.climber.extendFrontClimb(true);
					
					break;
					
				case BUTTON_BLACK:
					if (Devices.climber.isRearExtended())
						Devices.climber.retractRearClimb();
					else
						Devices.climber.extendRearClimb(true);
					
					break;
					
				case BUTTON_BLUE_RIGHT:
					//lift.setHeight(1000);
					if (Devices.pickup.isExtended())
						Devices.pickup.retract();
					else
						Devices.pickup.extend();
					
					break;
					
				case BUTTON_RED_RIGHT:
					if (Devices.lift.isHoldingHeight())
						Devices.lift.setHeight(-1);
					else
					{
						if (robot.isComp)
							Devices.lift.setHeight(1250);
						else
							Devices.lift.setHeight(1400);
					}
					
					break;
					
				case BUTTON_YELLOW:
					if (Devices.lift.isHoldingHeight())
						Devices.lift.setHeight(-1);
					else
					{
						if (robot.isComp)
							Devices.lift.setHeight(850);
						else
							Devices.lift.setHeight(724);
					}
					
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
					if (Devices.lift.isHoldingHeight())
						Devices.lift.setHeight(-1);
					else
						Devices.lift.setHeight(400);
					
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
					if (Devices.gearBox.isLowSpeed())
	    				Devices.gearBox.highSpeed();
	    			else
	    				Devices.gearBox.lowSpeed();

					break;
					
				case TOP_BACK:
					altDriveMode = !altDriveMode;
					SmartDashboard.putBoolean("AltDriveMode", altDriveMode);
					break;
					
				case TOP_MIDDLE:
					robot.cameraThread.addTargetRectangle(null);
					robot.cameraThread.setContours(null);
//					robot.vision.saveImages(true);
//
//					robot.vision.processImage(robot.cameraThread.getCurrentImage());
//					
//					if (robot.vision.targetVisible())
//					{
//						robot.cameraThread.addTargetRectangle(robot.vision.getTargetRectangles().get(0));
//						robot.cameraThread.addTargetRectangle(robot.vision.getTargetRectangles().get(1));
//						robot.cameraThread.setContours(robot.vision.getContours());
//						
//						int centerx = robot.vision.centerX();
//						int centery = robot.vision.centerY();
//						int offsetx = robot.vision.offsetX();
//						int offsety = robot.vision.offsetY();
//						
//						Util.consoleLog("centerx=%d offx=%d  centery=%d offy=%d  dist=%.1f", centerx, offsetx, centery, 
//										offsety, robot.vision.getDistance());
//						
//						robot.cameraThread.addTargetRectangle(new Rect(centerx-5,centery-5,10,10));
//					}

					robot.visionLL.processImage();
					
					if (robot.visionLL.targetVisible())
					{
						Rect	tRect = robot.visionLL.getTargetRectangle();
						
						robot.cameraThread.addTargetRectangle(tRect);
						
						Util.consoleLog("x=%d  y=%d  h=%d  w=%d", tRect.x, tRect.y, tRect.height, tRect.width);

						int centerx = robot.visionLL.centerX();
						int centery = robot.visionLL.centerY();
						int offsetx = robot.visionLL.offsetX();
						int offsety = robot.visionLL.offsetY();
						
						Util.consoleLog("centerx=%d offx=%d  centery=%d offy=%d  dist=%.1f", centerx, offsetx, centery, 
										offsety, robot.visionLL.getDistance());
						
						robot.cameraThread.addTargetRectangle(new Rect(centerx-5,centery-5,10,10));
					}
					
					break;
					
				case TOP_LEFT:
					driveToTargetLL();
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
//				case TRIGGER:
//					if (lift.isHatchReleased())
//						lift.grabHatch();
//					else
//						lift.releaseHatch();
//					
//					break;
					
				case TOP_LEFT:
					Devices.pickup.intakeBall();
					break;
					
				case TOP_RIGHT:
					Devices.pickup.spitBall();
					break;
					
				case TOP_MIDDLE:
					if (Devices.pickup.isAutoIntakeRunning())
						Devices.pickup.stopAutoIntake();
					else
						Devices.pickup.startAutoIntake();
					
					break;
				
				case TOP_BACK:
					Devices.lift.setHatchHeightAuto();
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
	
	// Handle NavX events.

	private class NavXListener implements NavXEventListener 
	{
		public void event( NavXEvent navXEvent )
		{
			if ( navXEvent.eventType == NavXEventType.collisionDetected)
				Util.consoleLog("collision detected = %3f", navXEvent.eventData);
		}
	}
	
	private void driveToTarget()
	{
		int		stopDistance = 260;
		
		Util.consoleLog("--------------------------------------------------------");

		autoTarget = true;
		Devices.robotDrive.setSafetyEnabled(false);

		do
		{
			robot.cameraThread.addTargetRectangle(null);
			robot.cameraThread.setContours(null);

			robot.vision.processImage(robot.cameraThread.getCurrentImage());
			
			if (!isTargetVisible(robot.vision.targetVisible())) break;
			
			robot.cameraThread.addTargetRectangle(robot.vision.getTargetRectangles().get(0));
			robot.cameraThread.addTargetRectangle(robot.vision.getTargetRectangles().get(1));
			robot.cameraThread.setContours(robot.vision.getContours());
			
			int centerx = robot.vision.centerX();
			int centery = robot.vision.centerY();
			int offsetx = robot.vision.offsetX();
			int offsety = robot.vision.offsetY();
			
			double curve = offsetx * (1 / 100.0) * (robot.vision.getDistance() / stopDistance);
					
			Util.consoleLog("centerx=%d offx=%d  centery=%d offy=%d  dist=%.1f  cur=%.3f", centerx, offsetx, centery, 
							offsety, robot.vision.getDistance(), curve);
			
			robot.cameraThread.addTargetRectangle(new Rect(centerx-5, centery-5, 10, 10));

			// Steer based on center of robot field of vision offset from target center (X axis). 
			// + offset is target right of center so we want to turn right. - offset is target left
			// of center so we want to turn left.
			// We invert since a - value causes left turn which would correct the right of center
			// robot heading. Adjust gain for reasonable correction effect, use distance to scale the
			// correction. At distance we want small corrections, as we approach target corrections
			// need to get larger. That would be 1/180. 
			
			//Devices.robotDrive.curvatureDrive(.25, offsetx * .003, false);
			//Devices.robotDrive.tankDrive(.30, .30);
			Devices.robotDrive.curvatureDrive(.25, curve, false);

			Timer.delay(.25);

		} while (robot.isEnabled() && robot.vision.getDistance() < stopDistance);
		
		Devices.robotDrive.stopMotor();
		
		autoTarget = false;
		Devices.robotDrive.setSafetyEnabled(true);

		Util.consoleLog("end driveToTarget");
	}
	
	private void driveToTargetLL()
	{
		int		stopDistance = 8;
		Rect	tRect;
		double	distance = 0;
		
		Util.consoleLog("--------------------------------------------------------");

		autoTarget = true;
		Devices.robotDrive.setSafetyEnabled(false);
		robot.cameraThread.setContours(null);
		robot.cameraThread.addTargetRectangle(null);

		do
		{
			robot.visionLL.processImage();
			
			if (!isTargetVisible(robot.visionLL.targetVisible())) break;
			
			tRect = robot.visionLL.getTargetRectangle();
			
			distance = robot.visionLL.getDistance();
			
			Util.consoleLog("x=%d  y=%d  h=%d  w=%d", tRect.x, tRect.y, tRect.height, tRect.width);
			
			int centerx = robot.visionLL.centerX();
			int centery = robot.visionLL.centerY();
			int offsetx = robot.visionLL.offsetX();
			int offsety = robot.visionLL.offsetY();
			
			double curve = offsetx * (1 / 23.0); // * (distance / stopDistance);
					
			Util.consoleLog("centerx=%d offx=%d  centery=%d offy=%d  dist=%.1f  cur=%.3f", centerx, offsetx, centery, 
							offsety, distance, curve);
			
			//robot.cameraThread.addTargetRectangle(new Rect(centerx-5, centery-5, 10, 10));

			// Steer based on center of robot field of vision offset from target center (X axis). 
			// + offset is target right of center so we want to turn right. - offset is target left
			// of center so we want to turn left.
			// Adjust gain for reasonable correction effect, use distance to scale the
			// correction. At distance we want small corrections, as we approach target corrections
			// need to get larger. That would be 1/8 (% area). 
			
			//Devices.robotDrive.curvatureDrive(.25, offsetx * .003, false);
			//Devices.robotDrive.tankDrive(.30, .30);
			Devices.robotDrive.curvatureDrive(.25, curve, false);

			Timer.delay(.25);

		} while (robot.isEnabled() && distance < stopDistance);
		
		Devices.robotDrive.stopMotor();
		
		autoTarget = false;
		Devices.robotDrive.setSafetyEnabled(true);

		Util.consoleLog("end driveToTargetLL");
	}
	
	// Allow for intermittent target visible indication. Target must be not visible
	// for .7 of a second. Not visible for less than .7 is converted to visible.
	// This counters the intermittent issue of the target filtering on both Grip and LL.
	
	private boolean isTargetVisible(boolean visible)
	{
		Util.consoleLog();
		
		if (visible)
			timeMarker = 0;
		else
		{
			if (timeMarker == 0)
			{
				visible = true;
				timeMarker = Util.timeStamp();
			}
			else if (Util.getElaspedTime(timeMarker) < .70)
				visible = true;
			else
			{
				visible = false;
				timeMarker = 0;
			}
		}
		
		return visible;
	}
}
