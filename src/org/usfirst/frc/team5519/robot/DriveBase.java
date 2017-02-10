package org.usfirst.frc.team5519.robot;

import edu.wpi.first.wpilibj.GenericHID;

public abstract class DriveBase {

	/**
	 * Just Drive! Under joystick command. 
	 * Code stolen from RobotDrive
	 * 
	 * @author GSN - 11/12/2016
	 */
	abstract void Drive(GenericHID stick);

	/**
	 * Drive using direct values. 
	 * Code stolen from RobotDrive
	 * 
	 * @author GSN - 11/12/2016
	 */
	 abstract void Drive(double moveValue, double rotateValue);
	
}
