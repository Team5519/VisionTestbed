package org.usfirst.frc.team5519.robot;

import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	public static boolean isLouise = true;		// BE SURE TO SET THIS TO FALSE IF RUNNING ARBOUR!!!

	public static int kFrontLeftMotorPort;
	public static int kFrontRightMotorPort;
	
	public static PWMSpeedController frontLeftMotor;
	public static PWMSpeedController frontRightMotor;
	
	public static void init() {
		
		if (isLouise) {
			// Assign definitions for LOUISE (Test Bot)
			kFrontLeftMotorPort = 1;	
			kFrontRightMotorPort = 3;	
			frontLeftMotor = new Talon(kFrontLeftMotorPort);
			frontRightMotor = new Talon(kFrontRightMotorPort);
		} else {
			// Assign definitions for FARADAY (Competition Bot)
			// FARADAY will be using VictorSPs and Sparks.
			kFrontLeftMotorPort = 1;	
			kFrontRightMotorPort = 3;	
			frontLeftMotor = new VictorSP(kFrontLeftMotorPort);
			frontRightMotor = new VictorSP(kFrontRightMotorPort);
		}

		// CY 1/17/2017
		// Test to see if this makes a difference.
		frontLeftMotor.enableDeadbandElimination(true);
		frontRightMotor.enableDeadbandElimination(true);
			
	}


}
