# Robot19
FRC Team 4450 2019 Robot Control program.

This is the 2019 competition robot control program created by the Olympia Robotics Federation (FRC Team 4450).
Operates the robot "TBA" for FRC game "DESTINATION DEEP SPACE".

## Instructions to setup development environment for Eclipse
1) Follow the instructions [here](http://wpilib.screenstepslive.com/s/4485/m/13809/l/599681-installing-eclipse-c-java) to setup the JDK, Eclipse, and the FRC plugins for Eclipse. Do not install the C++ portions.
2) Clone this repository to your local Eclipse workspace.
3) Import the project into Eclipse as an existing gradle project.
4) Edit the build.gradle file for the dependency line:
	compile "com.github.ORF-4450:RobotLib:v2.3" 
	and change the version of RobotLib to the current version available on Jitpack.com for ORF-4450.
5) If you change the version, right click on the project in Eclipse and select Gradle/Refresh Gradle Project.
6) Execute the run configuration called Build Robot19 to compile the project. First compile will take a while
	as gradle will download a number of components and libraries to the local cache. Note: the run configurations
	will need to have their Java Home path set to the specific jdk directory on your PC.

### If RobotLib gets an update:
1) Do steps 4, 5 & 6 above with the new version number. You do not need to download RobotLib, gradle will do it.
****************************************************************************************************************
Version 19.0

*	Cloned from Robot11. 2018 specific code removed, cleaned up to be baseline for 2019 season.

R. Corn
December 18, 2018
