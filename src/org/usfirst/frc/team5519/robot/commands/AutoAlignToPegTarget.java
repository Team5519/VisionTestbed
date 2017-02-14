package org.usfirst.frc.team5519.robot.commands;

import org.usfirst.frc.team5519.robot.Robot;
import org.usfirst.frc.team5519.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AutoAlignToPegTarget extends Command {
	
	private int sideTargetToBot;	// i.e. The bot is on this side of the target (we think)

    public AutoAlignToPegTarget(int side) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveBase);
    	requires(Robot.axisVision);
    	sideTargetToBot = side;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveBase.stopDead();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double angleToTarget = 0.0;
    	if (Robot.axisVision.isTargetLocked()) {
    		// Target is in vision so we just need to do some fine adjustment to the angle.
    		angleToTarget = Robot.axisVision.getTargetAngle();
    	} else {
    		// Target is not in vision so we have to rotate a bit in order to bring the target into view
    		// Direction of rotation depends on what side of the target (we think) we are on.
        	if (sideTargetToBot == RobotMap.START_POSITION_RIGHT) {
        		// We are on the right side of the target so we need to rotate left. (Counter Clockwise)
        		angleToTarget = +30.0;      		
        	} else {
        		// We are on the left side of the target so we need to rotate left.
        		// Note: If we started in the CENTRE we drifted for some reason so rotate right (Clockwise) (and hope) ( Clockwise).
        		angleToTarget = -30.0;      		
         	}
    	}
    	Robot.driveBase.rotateInPlace(angleToTarget);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Math.abs(Robot.axisVision.getTargetAngle()) < 1.0) {
    		// We are Within 1 degree which is good enough alignment for FRC work
            DriverStation.reportWarning("COMMAND AlignToPegTarget is ALIGNED to target.", false);
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
