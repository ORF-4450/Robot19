/**
 * Pattern for robot subsystems. All subsystems should extend this class
 * and implement these methods as appropriate.
 */
package Team4450.Robot19;

public abstract class SubSystem
{
	/*
	 * Called when robot is enabled. Subsystem should set it's starting
	 * configuration.
	 */
	abstract void enable();
	
	/*
	 * Called when robot is disabled. Subsystem should set it's stopped
	 * configuration. Any PID loops or threads should be stopped.
	 */
	abstract void disable();
	
	/*
	 * Called when the subsystem instance is released. All resources that
	 * need to be released should be released and singleton variable set
	 * to null.
	 */
	abstract void dispose();
	
	/*
	 * Common function where subsystem should set the DS indicators to the
	 * subsystem's current status as appropriate.
	 */
	abstract protected void updateDS();
}
