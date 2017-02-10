package org.usfirst.frc.team5519.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseAutonomous extends DriveBase {
	
	protected RobotDrive myDrive;
	private boolean underJoystickControl;
	
    // GyroSamples
    AHRS ahrs;
    private int autoCount;
    private double Kp;

    public void dumpAHRSData () {
        /* Display 6-axis Processed Angle Data                                      */
        SmartDashboard.putBoolean(  "AHRS/IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "AHRS/IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "AHRS/IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "AHRS/IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "AHRS/IMU_Roll",             ahrs.getRoll());
        
        /* Display tilt-corrected, Magnetometer-based heading (requires             */
        /* magnetometer calibration to be useful)                                   */
        
        SmartDashboard.putNumber(   "AHRS/IMU_CompassHeading",   ahrs.getCompassHeading());
        
        /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
        SmartDashboard.putNumber(   "AHRS/IMU_FusedHeading",     ahrs.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */
        
        SmartDashboard.putNumber(   "AHRS/IMU_TotalYaw",         ahrs.getAngle());
        SmartDashboard.putNumber(   "AHRS/IMU_YawRateDPS",       ahrs.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        
        SmartDashboard.putNumber(   "AHRS/IMU_Accel_X",          ahrs.getWorldLinearAccelX());
        SmartDashboard.putNumber(   "AHRS/IMU_Accel_Y",          ahrs.getWorldLinearAccelY());
        SmartDashboard.putBoolean(  "AHRS/IMU_IsMoving",         ahrs.isMoving());
        SmartDashboard.putBoolean(  "AHRS/IMU_IsRotating",       ahrs.isRotating());

        /* Display estimates of velocity/displacement.  Note that these values are  */
        /* not expected to be accurate enough for estimating robot position on a    */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially      */
        /* double (displacement) integration.                                       */
        
        SmartDashboard.putNumber(   "AHRS/Velocity_X",           ahrs.getVelocityX());
        SmartDashboard.putNumber(   "AHRS/Velocity_Y",           ahrs.getVelocityY());
        SmartDashboard.putNumber(   "AHRS/Displacement_X",       ahrs.getDisplacementX());
        SmartDashboard.putNumber(   "AHRS/Displacement_Y",       ahrs.getDisplacementY());
        
        /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
        /* NOTE:  These values are not normally necessary, but are made available   */
        /* for advanced users.  Before using this data, please consider whether     */
        /* the processed data (see above) will suit your needs.                     */
        
        SmartDashboard.putNumber(   "AHRS/RawGyro_X",            ahrs.getRawGyroX());
        SmartDashboard.putNumber(   "AHRS/RawGyro_Y",            ahrs.getRawGyroY());
        SmartDashboard.putNumber(   "AHRS/RawGyro_Z",            ahrs.getRawGyroZ());
        SmartDashboard.putNumber(   "AHRS/RawAccel_X",           ahrs.getRawAccelX());
        SmartDashboard.putNumber(   "AHRS/RawAccel_Y",           ahrs.getRawAccelY());
        SmartDashboard.putNumber(   "AHRS/RawAccel_Z",           ahrs.getRawAccelZ());
        SmartDashboard.putNumber(   "AHRS/RawMag_X",             ahrs.getRawMagX());
        SmartDashboard.putNumber(   "AHRS/RawMag_Y",             ahrs.getRawMagY());
        SmartDashboard.putNumber(   "AHRS/RawMag_Z",             ahrs.getRawMagZ());
        SmartDashboard.putNumber(   "AHRS/IMU_Temp_C",           ahrs.getTempC());
        
        /* Omnimount Yaw Axis Information                                           */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
        AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
        SmartDashboard.putString(   "AHRS/YawAxisDirection",     yaw_axis.up ? "Up" : "Down" );
        SmartDashboard.putNumber(   "AHRS/YawAxis",              yaw_axis.board_axis.getValue() );
        
        /* Sensor Board Information                                                 */
        SmartDashboard.putString(   "AHRS/FirmwareVersion",      ahrs.getFirmwareVersion());
        
        /* Quaternion Data                                                          */
        /* Quaternions are fascinating, and are the most compact representation of  */
        /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
        /* from the Quaternions.  If interested in motion processing, knowledge of  */
        /* Quaternions is highly recommended.                                       */
        SmartDashboard.putNumber(   "AHRS/QuaternionW",          ahrs.getQuaternionW());
        SmartDashboard.putNumber(   "AHRS/QuaternionX",          ahrs.getQuaternionX());
        SmartDashboard.putNumber(   "AHRS/QuaternionY",          ahrs.getQuaternionY());
        SmartDashboard.putNumber(   "AHRS/QuaternionZ",          ahrs.getQuaternionZ());
        
        /* Connectivity Debugging Support                                           */
        SmartDashboard.putNumber(   "AHRS/IMU_Byte_Count",       ahrs.getByteCount());
        SmartDashboard.putNumber(   "AHRS/IMU_Update_Count",     ahrs.getUpdateCount());

    }
    
    DriveBaseAutonomous() {
		myDrive = new RobotDrive(RobotMap.frontLeftMotor, RobotMap.frontRightMotor);
		underJoystickControl = true;
        //myDrive.setSafetyEnabled(true); 	// Ensure motor safety
        //myDrive.setExpiration(0.1);			// Suggested default safety timeout
        // GyroSamples
          try {
              /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
              /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
              /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
              ahrs = new AHRS(SPI.Port.kMXP); 
          } catch (RuntimeException ex ) {
              DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
          }
    }
    
    public void enableJoystickControl () {
    	underJoystickControl = true;
    }

    public void disableJoystickControl () {
    	underJoystickControl = false;
    }

	public void DriveStraight(double moveValue) {
		// GyroSamples
		ahrs.reset();
		autoCount = 0;
		Kp = 0.03;
		Kp = 0.3;
		
		// GyroSamples - Drive Straight for 4 seconds
		autoCount = autoCount + 1;
		if (autoCount <= 200) {
			// 4 seconds * 50Hz = 200 counts
			double angle = ahrs.getAngle();
			Drive(-0.5, angle*Kp);
			// Timer.delay(0.004);
			// Example Code to turn 90 degrees - Implement Twist???			
		} else {
			autoCount = 500;
			Drive(0.0, 0.0);
		}		
		
		
		// GyroSamples
		dumpAHRSData();

	}
	
	public void RotateInPlace(double targetAngle) {
		if (underJoystickControl) {
			return;
		}
        DriverStation.reportWarning("Drive Rotate Bot rotateAngle:  " + targetAngle, false);
    	double rotateValue = -0.300;
    	if (targetAngle < 0.0) {
    		rotateValue = -1.5 * rotateValue;
    	}
		//myDrive.drive(0.08, rotateValue);
		//myDrive.drive(0.0, rotateValue);
		myDrive.arcadeDrive(0.001, rotateValue, true);


	}


	/**
	 * Just Drive! Under joystick command. 
	 * Code stolen from RobotDrive
	 * 
	 * @author GSN - 11/12/2016
	 */
	public void Drive(GenericHID stick) {
		if (!underJoystickControl) {
			return;
		}
		SmartDashboard.putNumber(   "Joystick/Y-Axis Value",       stick.getY());
		SmartDashboard.putNumber(   "Joystick/X-Axis Value",       stick.getX());
 		double moveValue = -0.75 * stick.getY();
		// Correct left / right by inverting X-Axis values.
		double rotateValue = -0.6 * stick.getX();
		myDrive.arcadeDrive(moveValue, rotateValue, true);
	}

	/**
	 * Drive using direct values. 
	 * Code stolen from RobotDrive
	 * 
	 * @author GSN - 11/12/2016
	 */
	 public void Drive(double moveValue, double rotateValue) {
			if (!underJoystickControl) {
				return;
			}
	 		myDrive.arcadeDrive(moveValue, rotateValue);			 
	 }

}
