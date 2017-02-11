package org.usfirst.frc.team5519.robot.commands;

import org.usfirst.frc.team5519.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class AlignBot extends Command {

    public AlignBot() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
		requires(Robot.axisVision);

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveBase.rotateInPlace(Robot.axisVision.getTargetAngle());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//if (Math.abs(Robot.axisVision.getTargetDistance()) > 0.5) {
    	//	return false;
    	//}
    	if (Math.abs(Robot.axisVision.getTargetAngle()) > 1.0) {
    		return false;
    	}
        DriverStation.reportWarning("Align Bot is Positioned:  ", false);
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.driveBase.rotateInPlace(0.0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
