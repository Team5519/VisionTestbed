package org.usfirst.frc.team5519.robot.subsystems;

import org.usfirst.frc.team5519.robot.RobotMap;
import org.usfirst.frc.team5519.robot.commands.DriveWithJoystick;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBaseAutonomous extends Subsystem {
	
	protected RobotDrive myDrive;
	
    // GyroSamples
    private static final double kP = -0.03;
    AHRS ahrs;
    
    // CIMcoder am-3314a
    // 2 channel quadrature output, 20 pulses per channel per revolution
    private static final double kPulsesPerRotation = 20;
    private static final double kDistancePerPulse = 0.018; // in meters, 32.3 cm divided by 10 pulses
    private Encoder cimCoder;
    
    
    public DriveBaseAutonomous() {
		myDrive = new RobotDrive(RobotMap.frontLeftMotor, RobotMap.frontRightMotor);
        myDrive.setSafetyEnabled(true); 	// Ensure motor safety
        myDrive.setExpiration(0.1);			// Suggested default safety timeout
        
        cimCoder = new Encoder(RobotMap.kCIMcoderDioPortA,RobotMap.kCIMcoderDioPortB,false,Encoder.EncodingType.k2X);
        cimCoder.setDistancePerPulse(kDistancePerPulse);
        //cimCoder.setMinRate(minRate);
        //cimCoder.setSamplesToAverage(samplesToAverage);
        
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
    

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		setDefaultCommand(new DriveWithJoystick());
	}
    
    
    public void resetSensors() {
		ahrs.reset();
    	cimCoder.reset();
    }
    
    
   public double getDistanceTraveled() {
    	//return ahrs.getDisplacementY();
    	return cimCoder.getDistance();
    }
    
   
	public void driveStraight(double moveValue) {
		// GyroSamples
		double rotateValue = ahrs.getAngle()* kP;
		directDrive(moveValue, rotateValue);
	}
	
	public void rotateInPlace(double targetAngle) {
        DriverStation.reportWarning("Drive Rotate Bot rotateAngle:  " + targetAngle, false);
    	double rotateValue = -0.300;
    	if (targetAngle < 0.0) {
    		rotateValue = -1.3 * rotateValue;
            DriverStation.reportWarning("Rotate in place applying CORRECTION.", false);
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
	public void drive(GenericHID stick) {
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
	 public void directDrive(double moveValue, double targetAngle) {
		//DriverStation.reportWarning("Drive Rotate Bot rotateAngle:  " + targetAngle, false);
	    double rotateValue = -0.200;
	    if (targetAngle < 0.0) {
	    	rotateValue = -1.3 * rotateValue;
	        //DriverStation.reportWarning("Rotate in place applying CORRECTION.", false);
	    }
	 	myDrive.arcadeDrive(moveValue, rotateValue);			 
	 }
	 
	 public void stopDead() {
		if (ahrs.getVelocityY() > 0.1) {
		 	myDrive.arcadeDrive(-0.05, 0);			 			
		} else if (ahrs.getVelocityY() < -0.1) {
		 	myDrive.arcadeDrive(0.05, 0);			 			
		} else {
		 	myDrive.arcadeDrive(0, 0);			 			
		}
	 }


	
    public void dumpSensorData () {
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

	
}
