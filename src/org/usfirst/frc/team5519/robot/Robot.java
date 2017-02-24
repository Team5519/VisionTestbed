
package org.usfirst.frc.team5519.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team5519.robot.commands.AssistDeliverGear;
import org.usfirst.frc.team5519.robot.commands.AutoAlignToPegTarget;
import org.usfirst.frc.team5519.robot.commands.AutoDeliverGear;
import org.usfirst.frc.team5519.robot.commands.AutoDriveStraightDistance;
import org.usfirst.frc.team5519.robot.commands.AutoDriveToPegTarget;
import org.usfirst.frc.team5519.robot.subsystems.AxisVision;
import org.usfirst.frc.team5519.robot.subsystems.DriveBaseAutonomous;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	public static OI oi;
	
    public static DriveBaseAutonomous driveBase;
	public static AxisVision axisVision;

	Command autonomousCommand;
	//SendableChooser<Command> chooser = new SendableChooser<>();
	//SendableChooser<String> chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		RobotMap.init ();
		driveBase = new DriveBaseAutonomous();
		axisVision = new AxisVision();
		oi = new OI();
		/**
		chooser.addObject("Default Auto", new AutoDeliverGearRight());
		chooser.addDefault("Align To Peg", new AutoAlignToPegTarget(RobotMap.START_POSITION_LEFT));
		chooser.addObject("Drive To Peg", new AutoDriveToPegTarget());
		chooser.addObject("Drive Straight", new AutoDriveStraightDistance(2.0));
		SmartDashboard.putData("Select Auto", chooser);
		*/
		SmartDashboard.putString("Auto List", "Test Autio 1");
		SmartDashboard.putString("Auto List", "Test Autio 2");
				
		axisVision.initCameraHardware();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		//autonomousCommand = chooser.getSelected();
		String autoSelected = SmartDashboard.getString("Auto Selector","Default");
        DriverStation.reportWarning("AUTONOMOUS COMMAND = " + autoSelected, false);
		switch(autoSelected) { 
			case "L1": 
				autonomousCommand = new AutoDeliverGear(RobotMap.START_POSITION_LEFT); 
				break; 
			case "R1": 
				autonomousCommand = new AutoDeliverGear(RobotMap.START_POSITION_RIGHT); 
				break; 
			case "C1": 
				autonomousCommand = new AssistDeliverGear(); 
				break; 
			case "Default": 
			default:
				autonomousCommand = new AutoDriveStraightDistance(2.0); 
				break; 
		}
		if (autonomousCommand != null)
			autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null)
			autonomousCommand.cancel();
		driveBase.stopDead();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		driveBase.dumpSensorData();
		//RobotMap.sweeperMotor.set(1.0);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
