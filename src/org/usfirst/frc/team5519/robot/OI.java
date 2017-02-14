package org.usfirst.frc.team5519.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;

import org.usfirst.frc.team5519.robot.commands.AlignBot;
import org.usfirst.frc.team5519.robot.commands.AutoAlignToPegTarget;
import org.usfirst.frc.team5519.robot.commands.AutoDeliverGearRight;
import org.usfirst.frc.team5519.robot.commands.AutoDriveStraightDistance;
import org.usfirst.frc.team5519.robot.commands.AutoDriveToPegTarget;
import org.usfirst.frc.team5519.robot.commands.CameraToggleSettings;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
	
	public static final int kDriveStickPort = 0;	
	public static Joystick driveStick;
	
	public static final int kDeliverGearButtonNumber = 1;
	public static Button deliverGearButton;
	public static final int kAlignToGearButtonNumber = 7;
	public static Button alignToGearButton;
	public static final int kDriveToGearButtonNumber = 10;
	public static Button driveToGearButton;
	public static final int kDriveStraightGearButtonNumber = 11;
	public static Button driveStraightGearButton;

	public static final int kToggleVisionButtonNumber = 2;
	public static Button toggleVisionButton;


	public OI() {
		OI.driveStick = new Joystick(kDriveStickPort);
		
		Command deliverGear = new AutoDeliverGearRight();
		OI.deliverGearButton = new JoystickButton(OI.driveStick,kDeliverGearButtonNumber);
		OI.deliverGearButton.toggleWhenPressed(deliverGear);

		Command alighToGear = new AutoAlignToPegTarget(RobotMap.START_POSITION_LEFT);
		OI.alignToGearButton = new JoystickButton(OI.driveStick,kAlignToGearButtonNumber);
		OI.alignToGearButton.toggleWhenPressed(alighToGear);

		Command driveToGear = new AutoDriveToPegTarget();
		OI.driveToGearButton = new JoystickButton(OI.driveStick,kDriveToGearButtonNumber);
		OI.driveToGearButton.toggleWhenPressed(driveToGear);

		Command driveStraight = new AutoDriveStraightDistance(2.0);
		OI.driveStraightGearButton = new JoystickButton(OI.driveStick,kDriveStraightGearButtonNumber);
		OI.driveStraightGearButton.toggleWhenPressed(driveStraight);

		Command toggleVision = new CameraToggleSettings();
		OI.toggleVisionButton = new JoystickButton(OI.driveStick,kToggleVisionButtonNumber);
		OI.toggleVisionButton.toggleWhenPressed(toggleVision);

	}
	
}
