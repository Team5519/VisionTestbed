package org.usfirst.frc.team5519.robot.commands;

import org.usfirst.frc.team5519.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveToPegTarget extends Command {
	
	private static final double  kP = 0.05;		// Proportionality constant for angle 
	private static final double  kFMV = 0.3;	// Initial FAST move value
	private static final double  kSMV = 0.15;	// Initial SLOW move value
	private double moveValue;
	private double rotateValue;
	
	private int sanityCounter;							// Sanity check for unlocked target condition
	private static final int  kMaxSanityCount = 5;	// Maximum iterations without target lock


    public AutoDriveToPegTarget() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	requires(Robot.axisVision);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	moveValue = kFMV;
    	rotateValue = 0.0;
    	sanityCounter = 0;
    	Robot.driveBase.stopDead();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.axisVision.isTargetLocked()) {
    		sanityCounter = 0;
    		rotateValue = Robot.axisVision.getTargetAngle() * kP;
        	if (Robot.axisVision.getTargetDistance() < 1.0) {
        		// Slow down for last 1.0 meters
        		moveValue = kSMV;
        	}
           	Robot.driveBase.directDrive(moveValue, rotateValue);
    	} else {
    		// Target is NOT locked so increment sanity and use last known good values 
    		++sanityCounter;
           	Robot.driveBase.directDrive(moveValue, rotateValue);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.axisVision.getTargetDistance() < 0.5) {
    		// We are Within 0.5 meters which is close enough for finer adjustments to take over
            DriverStation.reportWarning("COMMAND DriveToPegTarget is POSITIONED CLOSE to target.", false);
    		return true;
    	}
    	if (sanityCounter >= kMaxSanityCount) {
    		// Target was NOT locked for several iterations so shut things down
            DriverStation.reportWarning("COMMAND DriveToPegTarget target SANITY EXPIRED.", false);
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
       	Robot.driveBase.stopDead();
   }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
