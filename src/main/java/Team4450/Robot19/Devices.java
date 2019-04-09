package Team4450.Robot19;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import Team4450.Lib.JoyStick;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.NavX;
import Team4450.Lib.SRXMagneticEncoderRelative;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import Team4450.Lib.ValveSA;
import Team4450.Lib.JoyStick.JoyStickButtonIDs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;

public class Devices
{
	  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	  public static WPI_TalonSRX		LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;
	  
	  public static WPI_VictorSPX		leftWinch, rightWinch, pickupMotor, ballSpit;	
	  
	  public static CANSparkMax			leftSpark, rightSpark;
	  
	  public static VictorSP			hatchWinch;
	  
	  public static DifferentialDrive		robotDrive;
	  public static	SpeedControllerGroup 	hDrive;
	  public static SpeedControllerGroup	winchDrive;
	  
	  public static JoyStick			rightStick = null;
	  public static JoyStick			leftStick = null;
	  public static JoyStick			utilityStick = null;
	  public static LaunchPad			launchPad = null;

	  public final static Compressor	compressor = new Compressor(0);		// Compressor class represents the PCM.

	  public final static ValveDA		highLowValve = new ValveDA(0);			// For gearbox.
	  public final static ValveDA		frontClimbValve = new ValveDA(2);		// For front lift.
	  public final static ValveDA		rearClimbValve = new ValveDA(6);	//(4);		// For rear lift.
	  public final static ValveDA		pickupValve = new ValveDA(1,0);			// For pickup arm.
	  public final static ValveDA		hatchReleaseValve = new ValveDA(4);	//(6);
	  //public final static ValveSA		hatchReleaseValve = new ValveSA(1, 2);	// release hatch.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static PowerDistributionPanel	pdp = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;

	  public static boolean				winchEncoderEnabled = true;

	  // Encoder (regular type) is plugged into dio port n:
	  // orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	  public final static Encoder		winchEncoder = new Encoder(0, 1, true, EncodingType.k4X);
	  public final static Encoder		hatchEncoder = new Encoder(2, 3, true, EncodingType.k4X);
	  
	  public static DigitalInput		winchSwitch = new DigitalInput(4);
	  public static DigitalInput		ballSwitch = new DigitalInput(5);
	  
	  public static AnalogInput			ballSensor = new AnalogInput(1);

	  // SRX magnetic encoder plugged into a CAN Talon.
	  public static SRXMagneticEncoderRelative	leftEncoder, rightEncoder;
	  
	  private static boolean			talonBrakeMode;
	  
	  // Private constructor prevents creation of any instances of this "static" class.
	  
	  private Devices() {}
	  
	  // Initialize motor controllers, groups and encoders.
	  
	  public static void configureDevices(Robot robot)
	  {
		  Util.consoleLog();

		  // Create the drive Talons.
		  LFCanTalon = new WPI_TalonSRX(1);
		  LRCanTalon = new WPI_TalonSRX(2);
		  RFCanTalon = new WPI_TalonSRX(3);	
		  RRCanTalon = new WPI_TalonSRX(4);	
		  
	      // Initialize CAN Talons and write status to log so we can verify
	      // all the Talons are connected.
	      InitializeCANTalon(LFCanTalon);
	      InitializeCANTalon(LRCanTalon);
	      InitializeCANTalon(RFCanTalon);
	      InitializeCANTalon(RRCanTalon);

	      // Configure CAN Talons with correct inversions.
	      LFCanTalon.setInverted(true);
		  LRCanTalon.setInverted(true);
		  
		  RFCanTalon.setInverted(true);
		  RRCanTalon.setInverted(true);

	      // Turn on brake mode for drive CAN Talons.
	      SetCANTalonBrakeMode(true);
	      
	      // Setup the SpeedControllerGroups for the left and right set of motors.
	      // Groups allow 4 motors to be used with DifferentialDrive. You can also
	      // use follower mode where one motor on a side follows the other on the
	      // same side.
	      //SpeedControllerGroup LeftGroup = new SpeedControllerGroup(LFCanTalon, LRCanTalon);
		  //SpeedControllerGroup RightGroup = new SpeedControllerGroup(RFCanTalon, RRCanTalon);
		  
		  // Since encoder is on rear motor talon, set front talons to follow the rears when
		  // we do closed loop control using the rear talons. In closed loop control the
		  // talon is set to some setpoint and will move to that point using the encoder or
	      // you set a velocity setpoint and talon will run at that velocity. This is
		  // onboard PID control.
	      
	      // For 2019 robot, put rear talons into a differential drive object and set the
	      // front talons to follow the rears. Not going to get to closed loop control...
		  
		  LFCanTalon.set(ControlMode.Follower, LRCanTalon.getDeviceID());
		  RFCanTalon.set(ControlMode.Follower, RRCanTalon.getDeviceID());
		  
		  //robotDrive = new DifferentialDrive(LeftGroup, RightGroup);
		  robotDrive = new DifferentialDrive(LRCanTalon, RRCanTalon);
		  
		  // Configure SRX encoders as needed for measuring velocity and distance. 
		  // 5.8 is wheel diameter in inches. Adjust for each years robot.
		  rightEncoder = new SRXMagneticEncoderRelative(RRCanTalon, 5.8);
		  leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, 5.8);
		  
		  leftEncoder.setInverted(true);
		  leftEncoder.setMaxPeriod(1);
		  rightEncoder.setMaxPeriod(1);
		  leftEncoder.setPIDSourceType(PIDSourceType.kRate);
		  leftEncoder.setPIDRateType(PIDRateType.RPM);
		  rightEncoder.setPIDSourceType(PIDSourceType.kRate);
		  rightEncoder.setPIDRateType(PIDRateType.RPM);
		  
		  // Create Spark controllers for H drive.
		  leftSpark = new CANSparkMax(5, MotorType.kBrushless);
		  rightSpark = new CANSparkMax(6, MotorType.kBrushless);
		  
		  leftSpark.setIdleMode(IdleMode.kBrake);
		  rightSpark.setIdleMode(IdleMode.kBrake);
		  
		  leftSpark.setOpenLoopRampRate(2.0);
		  rightSpark.setOpenLoopRampRate(2.0);

		  // Setup a SpeedControllerGroup for the left and right H drive motors.
	      hDrive = new SpeedControllerGroup(leftSpark, rightSpark);
	      
	      hDrive.setInverted(true);
	      
	      // Create the Victor controllers.
		  leftWinch = new WPI_VictorSPX(7);
		  rightWinch = new WPI_VictorSPX(8);
		  pickupMotor = new WPI_VictorSPX(9);
		  ballSpit = new WPI_VictorSPX(10);
		  hatchWinch = new VictorSP(0);
		  
		  if (robot.isClone)
			  winchEncoder.setReverseDirection(false);
		  else
			  winchEncoder.setReverseDirection(true);
			  
		  //hatchEncoder.setReverseDirection(false);
		  
		  hatchWinch.setInverted(true);
		  
		  leftWinch.setNeutralMode(NeutralMode.Brake);
		  rightWinch.setNeutralMode(NeutralMode.Brake);
		  
		  rightWinch.setInverted(true);
		  
		  pickupMotor.setNeutralMode(NeutralMode.Brake);
		  ballSpit.setNeutralMode(NeutralMode.Brake);

		  // Setup a SpeedControllerGroup for the left and right winch drive motors.
	      winchDrive = new SpeedControllerGroup(leftWinch, rightWinch);	     
	     
	      // Increase valve slide time for valve solenoids as for some reason the valves on the
	      // 2019 comp robot stick.
	      //unusedValve.solenoidSlideTime = .10;
	      hatchReleaseValve.solenoidSlideTime = .25;
	      highLowValve.solenoidSlideTime = .10;
   		  rearClimbValve.solenoidSlideTime = .10;
   		  pickupValve.solenoidSlideTime = .10;
   		 
   		  // Create launch pad with all buttons monitored and auto start of monitoring loop.
   		  // Will add event handler in Teleop class.
 		  launchPad = new LaunchPad(new Joystick(3));

 		  // Create our JoyStick classes for each JS with selective button monitoring. Must start monitoring
 		  // loop manually when not adding all buttons. Will add event handler in Teleop class.
 		  leftStick = new JoyStick(new Joystick(0), "LeftStick", JoyStickButtonIDs.TRIGGER);
 		  //Example on how to track an additional button:
 		  leftStick.AddButton(JoyStickButtonIDs.TOP_BACK);
 		  leftStick.Start();

 		  rightStick = new JoyStick(new Joystick(1), "RightStick", JoyStickButtonIDs.TRIGGER);
 		  rightStick.AddButton(JoyStickButtonIDs.TOP_MIDDLE);
 		  rightStick.Start();

 		  // Create utility stick with all buttons monitored and auto start.
 		  utilityStick = new JoyStick(new Joystick(2), "UtilityStick");
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(WPI_TalonSRX talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults(0); //0ms means no blocking.
	  }
	  
	  // Set neutral behavior of drive CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(boolean brakeMode)
	  {
		  Util.consoleLog("brakes on=%b", brakeMode);
		  
		  SmartDashboard.putBoolean("Brakes", brakeMode);

		  talonBrakeMode = brakeMode;
		  
		  NeutralMode newMode;
		  
		  if (brakeMode) 
			  newMode = NeutralMode.Brake;
		  else 
			  newMode = NeutralMode.Coast;
		  
		  LFCanTalon.setNeutralMode(newMode);
		  LRCanTalon.setNeutralMode(newMode);
		  RFCanTalon.setNeutralMode(newMode);
		  RRCanTalon.setNeutralMode(newMode);
	  }
	  
	  public static boolean isBrakeMode()
	  {
		  return talonBrakeMode;
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is number of seconds from zero to full output.
	  // zero disables.
	  
	  public static void SetCANTalonRampRate(double seconds)
	  {
		  Util.consoleLog("%.2f", seconds);
		  
		  LFCanTalon.configOpenloopRamp(seconds, 0);
		  LRCanTalon.configOpenloopRamp(seconds, 0);
		  RFCanTalon.configOpenloopRamp(seconds, 0);
		  RRCanTalon.configOpenloopRamp(seconds, 0);
	  }
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getOutputCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getOutputCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getOutputCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getOutputCurrent()
				  );
	  }
}
