package org.usfirst.frc.team5519.robot.commands;

import org.usfirst.frc.team5519.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CameraToggleSettings extends Command {

	
    public CameraToggleSettings() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.axisVision);
     }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.axisVision.toggelCameraSetting();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
