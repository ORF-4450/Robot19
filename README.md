# Robot19-RAC
FRC Team 4450 2019 Robot Control program.

##This Mr. Corn's example.

This is the 2019 competition robot control program created by the Olympia Robotics Federation (FRC Team 4450).
Operates the robot "TBA" for FRC game "DESTINATION DEEP SPACE".

## Instructions to setup development environment for VS Code
1) Follow the instructions [here](https://wpilib.screenstepslive.com/s/currentCS/m/java) to setup the JDK, Visual Studio Code, the FRC plugins and tools. Do not install the C++ portion. You do not need the FRC Update Suite to compile code.
2) Clone this repository to local folder.
3) Open that folder in Visual Studio Code.
4) Build the project using the WPILib command from the WPILib commands list.

### If RobotLib gets an update:
1) download the RobotLib.json file from the RobotLib Github repo and drop it into the vendordeps folder inside the project folder. Build the project.
****************************************************************************************************************
Version 19.4-RAC-Abstract

*	Fixed some bugs introduced by converting subsystem classes to singletons.
*	Added abstract SubSystem class to be extended by all sub system classes in order to better standardize them
*	and void the problems that can occur when sub system classes are created at start up and hang around until
*	the power is turned off.

R. Corn, October 2019

Version 19.3-RAC2

*	Convert all subsystem classes to singletons and create them at code startup. Minimize the initialization
*	needed when starting auto or tele so the robot starts as quickly as possible after enabled. This created
*	some issues since the subsystem classes stick around instead of being created/disposed on each enable
*	disable cycle. Additional work to shorten the initialization code paths.

R. Corn, Summer 2019

Version 19.2-RAC

*	Add Lift and Pickup classes in anticipation of those systems.
*	Convert Autonomous, Teleop and GearBox classes to Singleton pattern.

R. Corn January 29, 2019

Version 19.1

*	Major updates for compatibility with all the changes made by FIRST in 2019 Kickoff release.
*	Switch from Eclipse to VSCode IDE.

S. Flo, R.Corn
January 7, 2019

Version 19.0

*	Cloned from Robot11. 2018 specific code removed, cleaned up to be baseline for 2019 season.

R. Corn
December 18, 2018
