package org.usfirst.frc.team5519.robot.commands;

import org.usfirst.frc.team5519.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoDriveToPegTarget extends Command {
	
	private static final double  kFMV = 0.5;					// Initial FAST move value
	private static final double  kSMV = 0.25;					// Initial SLOW move value
	private static final double  kMIN_TARGET_DISTANCE = 0.75;	// Target distance limit for isFinished
	private static final double  kCLOSE_TARGET_DISTANCE = 1.5;	// Target distance limit for slower approach speed
	private double moveValue;
	private double rotateAngle;
	
	private int sanityCounter;							// Sanity check for unlocked target condition
	private static final int  kMaxSanityCount = 50;	// Maximum iterations without target lock


    public AutoDriveToPegTarget() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	requires(Robot.axisVision);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	moveValue = kFMV;
    	rotateAngle = 0.0;
    	sanityCounter = 0;
    	Robot.driveBase.stopDead();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.axisVision.isTargetLocked()) {
    		sanityCounter = 0;
    		//rotateValue = Robot.axisVision.getTargetAngle() * kP;
    		rotateAngle = Robot.axisVision.getTargetAngle();
        	if (Robot.axisVision.getTargetDistance() < kCLOSE_TARGET_DISTANCE) {
        		// Slow down for last 1.0 meters
        		moveValue = kSMV;
        	}
            //DriverStation.reportWarning("COMMAND DriveToPegTarget is LOCKED on target." + rotateAngle, false);
           	Robot.driveBase.directDrive(moveValue, rotateAngle);
    	} else {
    		// Target is NOT locked so increment sanity and use last known good values 
    		sanityCounter = sanityCounter +1;
           	//Robot.driveBase.directDrive(kSMV, rotateValue);
    		Robot.driveBase.directDrive(0, 0);
            //DriverStation.reportWarning("COMMAND DriveToPegTarget is NOT LOCKED." + rotateAngle, false);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.axisVision.getTargetDistance() < kMIN_TARGET_DISTANCE) {
    		// We are Within 0.5 meters which is close enough for finer adjustments to take over
            DriverStation.reportWarning("COMMAND DriveToPegTarget is POSITIONED CLOSE to target.", false);
    		return true;
    	}
    	if (sanityCounter >= kMaxSanityCount) {
    		// Target was NOT locked for several iterations so shut things down
            //DriverStation.reportWarning("COMMAND DriveToPegTarget target SANITY EXPIRED.", false);
    		//return true;
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
