/**
 * 2019 competition robot code.
 *
 * For Robot "tba" built for FRC game "DESTINATION DEEP SPACE".
*/

package Team4450.Robot19;

import java.util.Properties;

import Team4450.Lib.*;
import Team4450.Robot19.Devices;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file.
 */
	
@SuppressWarnings("deprecation")
public class Robot extends SampleRobot 
{
  static final String  	PROGRAM_NAME = "RAC19-05.29.19-01";

  public Properties		robotProperties;
  
  public boolean		isClone = false, isComp = false;
  
  public RobotState		currentRobotState = RobotState.boot, lastRobotState = RobotState.boot;
    	
  DriverStation.Alliance	alliance;
  int                       location, matchNumber;
  String					eventName, gameMessage;
    
  Thread               	monitorBatteryThread, monitorPDPThread;
  MonitorCompressor		monitorCompressorThread;
  CameraFeed			cameraThread;
  Vision				vision;
  VisionLL				visionLL;
  
  Teleop 				teleOp;
  Autonomous 			autonomous;
  
  // Constructor.
  
  public Robot()
  {	
	// Set up our custom logger.
	 
	try
	{
		Util.CustomLogger.setup();
		
		// Set Java to catch any uncaught exceptions and record them in our log file. 
		
		Thread.setDefaultUncaughtExceptionHandler(new Thread.UncaughtExceptionHandler() 
		{
			public void uncaughtException(Thread t, Throwable e)
			{
		        Util.consoleLog("Uncaught exception from thread " + t);
		        Util.logException(e);
		    }

		});

		Util.consoleLog(PROGRAM_NAME);

    	Util.consoleLog("RobotLib=%s", LibraryVersion.version);

    	// Create SendableVersion object so it will be sent to the dashboard and also
    	// log some of it's information.
    	
    	SendableVersion.INSTANCE.init(PROGRAM_NAME);
  		
   		Util.consoleLog("compiled by %s at %s (branch=%s, commit=%s)", SendableVersion.INSTANCE.getUser(),
   				SendableVersion.INSTANCE.getTime(), SendableVersion.INSTANCE.getBranch(),
   				SendableVersion.INSTANCE.getCommit());

    }
    catch (Exception e) {Util.logException(e);}
  }
    
  // Initialization, called at class start up.
  
  public void robotInit()
  {
   	try
    {
   		Util.consoleLog();
        
        currentRobotState = RobotState.init;

   		LCD.clearAll();
   		LCD.printLine(1, "Mode: RobotInit");
      
   		// Read properties file from RoboRio "disk".
      
   		robotProperties = Util.readProperties();
      
   		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;

		// Send program version to the dashboard.
   		
		SmartDashboard.putString("Program", PROGRAM_NAME);
   		
   		SmartDashboard.putData(SendableVersion.INSTANCE);
   		
   		// Set compressor enabled switch on dashboard from properties file.
   		// Later code will read that setting from the dashboard and turn 
   		// compressor on or off in response to dashboard setting.
   		
   		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

   		// Reset PDB & PCM sticky faults.
      
   		Devices.pdp.clearStickyFaults();
   		Devices.compressor.clearAllPCMStickyFaults();
   		
   		// Configure the robot's devices (motor controllers, valves, encoders etc.).
   		
   		Devices.configureDevices(this);

   		// Configure starting motor safety;
   		
   		Devices.robotDrive.stopMotor();
   		Devices.robotDrive.setSafetyEnabled(false);
   		Devices.robotDrive.setExpiration(0.1);
             
   		// Create NavX object here since must done before CameraFeed is created (don't remember why).
   		// Navx calibrates at power on and must complete before robot moves. Takes 12 seconds.

   		Devices.navx = NavX.getInstance(NavX.PortType.SPI);
   		
   		//Devices.navx.dumpValuesToNetworkTables();

   		// Add navx as a Sendable. Updates the Gyro indicator automatically when 
   		// SmartDashboard.updateValues() is called elsewhere.
   		
   		SmartDashboard.putData("Gyro2", Devices.navx);

   		// Start the battery, compressor, PDP and camera feed monitoring Tasks.

   		monitorBatteryThread = MonitorBattery.getInstance();
   		monitorBatteryThread.start();

   		monitorCompressorThread = MonitorCompressor.getInstance(Devices.pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
   		
   		//monitorPDPThread = MonitorPDP.getInstance(ds, PDP);
   		//monitorPDPThread.start();

   		// Start camera server thread using our class for usb cameras.
      
       	cameraThread = CameraFeed.getInstance(); 
       	cameraThread.start();
       	
       	vision = Vision.getInstance(this);
       	
       	visionLL = VisionLL.getInstance(this);
       	
       	// Configure autonomous program choices sendable chooser.
       	
       	Autonomous.setAutoChoices();
   		
   		Devices.navx.dumpValuesToNetworkTables();
   		
   		if (Devices.navx.isConnected())
   			Util.consoleLog("NavX version=%s", Devices.navx.getAHRS().getFirmwareVersion());
   		else
   		{
   			Exception e = new Exception("NavX is NOT connected!");
   			Util.logException(e);
   		}
   		
       	lastRobotState = currentRobotState;
       	
   		Util.consoleLog("end");
    }
    catch (Exception e) {Util.logException(e);}
  }
  
  // Called when robot is disabled.
  
  public void disabled()
  {
	  try
	  {
		  Util.consoleLog();
          
          currentRobotState = RobotState.disabled;

		  LCD.printLine(1, "Mode: Disabled");
		  
		  // Reset driver station LEDs.

		  SmartDashboard.putBoolean("Disabled", true);
		  SmartDashboard.putBoolean("Auto Mode", false);
		  SmartDashboard.putBoolean("Teleop Mode", false);
		  SmartDashboard.putBoolean("FMS", Devices.ds.isFMSAttached());
		  SmartDashboard.putBoolean("AutoTarget", false);
		  SmartDashboard.putBoolean("TargetLocked", false);
		  SmartDashboard.putBoolean("Overload", false);
		  SmartDashboard.putNumber("AirPressure", 0);
		  SmartDashboard.putBoolean("AltDriveMode", false);
		  SmartDashboard.putBoolean("SteeringAssist", false);

		  //lastRobotState = currentRobotState;

		  Util.consoleLog("end");
	  }
	  catch (Exception e) {Util.logException(e);}
  }
  
  // Called at the start of Autonomous period.
  
  public void autonomous() 
  {
      try
      {
    	  Util.consoleLog();
          
          currentRobotState = RobotState.auto;

          // Motor safety turned off during autonomous. 		
          Devices.robotDrive.setSafetyEnabled(false);

    	  LCD.clearAll();
    	  LCD.printLine(1, "Mode: Autonomous");
            
    	  SmartDashboard.putBoolean("Disabled", false);
    	  SmartDashboard.putBoolean("Auto Mode", true);
        
    	  getMatchInformation();
    	  
    	  // This code turns off the automatic compressor management if requested by DS.
    	  Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));
  	     
          //Devices.unusedValve.Close();

    	  // Reset persistent fault flags in control system modules.
    	  Devices.pdp.clearStickyFaults();
    	  Devices.compressor.clearAllPCMStickyFaults();
             
    	  // Start autonomous process contained in the Autonomous class.
        
    	  autonomous = Autonomous.getInstance(this);
        
    	  autonomous.execute();

    	  lastRobotState = currentRobotState;
      }
      catch (Exception e) {Util.logException(e);}
      
      finally
      {
      	  if (autonomous != null) autonomous.dispose();

      	  SmartDashboard.putBoolean("Auto Mode", false);
      	  Util.consoleLog("end");
      }
  }

  // Called at the start of the teleop period.
  
  public void operatorControl() 
  {
      try
      {
    	  Util.consoleLog();
          
          currentRobotState = RobotState.teleop;

          // Motor safety turned off during initialization.
          Devices.robotDrive.setSafetyEnabled(false);

    	  LCD.clearAll();
      	  LCD.printLine(1, "Mode: Teleop");
            
      	  SmartDashboard.putBoolean("Disabled", false);
      	  SmartDashboard.putBoolean("Teleop Mode", true);
        
      	  getMatchInformation();
      	  
    	  // Reset persistent fault flags in control system modules.
          Devices.pdp.clearStickyFaults();
          Devices.compressor.clearAllPCMStickyFaults();

          // This code turns off the automatic compressor management if requested by DS.
          Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));
 	     
          //Devices.unusedValve.Close();

          // Start operator control process contained in the Teleop class.
        
          teleOp = Teleop.getInstance(this);
       
          teleOp.OperatorControl();

          lastRobotState = currentRobotState;
       }
       catch (Exception e) {Util.logException(e);} 
       
       finally
       {
           if (teleOp != null) teleOp.dispose();
         	
           Util.consoleLog("end");
       }
  }
    
  public void test() 
  {
	  Util.consoleLog();
	  
	  currentRobotState = RobotState.test;

      lastRobotState = currentRobotState;
  }

  // Start usb camera server for single camera.
  
  public void StartUSBCameraServer(String cameraName, int device)
  {
	  Util.consoleLog("%s:%d", cameraName, device);

      CameraServer.getInstance().startAutomaticCapture(cameraName, device);
  }
  
  // Get and log information about the current match from the FMS or DS.
  
  public void getMatchInformation()
  {
  	  alliance = Devices.ds.getAlliance();
  	  location = Devices.ds.getLocation();
	  eventName = Devices.ds.getEventName();
	  matchNumber = Devices.ds.getMatchNumber();
	  gameMessage = Devices.ds.getGameSpecificMessage();
    
      Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, Devices.ds.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
  }
  
  public enum RobotState
  {
	  boot,
	  init,
	  disabled,
	  auto,
	  teleop,
	  test
  }
}
