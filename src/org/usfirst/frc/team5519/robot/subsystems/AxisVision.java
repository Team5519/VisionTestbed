package org.usfirst.frc.team5519.robot.subsystems;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.usfirst.frc.team5519.robot.vision.PegTarget;
import org.usfirst.frc.team5519.robot.vision.PegVisionPipeline;


/**
 *
 */
public class AxisVision extends Subsystem {

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private AxisCamera camera;
	private boolean inVisionMode;
	private Thread visionThread;
	
	private final Object imgLock = new Object();
	private double targetAngle = 0.0;
	private double targetDistance = 99.9;
	private boolean isTargetLocked = false;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void setCameraForTeleop() {
    	camera.setResolution(IMG_WIDTH, IMG_HEIGHT); 
    	inVisionMode = false;
    }
    
    public void setCameraForVision() {
    	camera.setResolution(IMG_WIDTH, IMG_HEIGHT); 
    	inVisionMode = true;
    }
    
    public void initCameraHardware() {
    	camera = CameraServer.getInstance().addAxisCamera("Axis Stream","axis-camera");
    	setCameraForVision();
    	
        visionThread = new Thread(() -> {
            Mat snapshot = new Mat();
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Peg Target", IMG_WIDTH, IMG_HEIGHT);
            PegVisionPipeline pipeline = new PegVisionPipeline();    
            while(!Thread.interrupted()) {
                cvSink.grabFrame(snapshot);
                pipeline.process(snapshot);
                ArrayList<MatOfPoint> contourDetections = pipeline.filterContoursOutput();
                SmartDashboard.putNumber("Target/Number of Pipeline Contours", contourDetections.size());
                PegTarget pegTarget = new PegTarget(contourDetections);
                synchronized (imgLock) {
                    targetAngle = pegTarget.estimateAngle();
                    targetDistance = pegTarget.estimateDistance();
                    isTargetLocked = pegTarget.isEstablished();
                }
                //snapshot = pipeline.hslThresholdOutput();
                pegTarget.dumpStatistics();
                pegTarget.drawBoxOnImage(snapshot);
                outputStream.putFrame(snapshot);
                //Timer.delay(0.1);
            }
        });
        visionThread.start();

    }
    
    public double getTargetAngle() {
    	double angle = 0.0;
        synchronized (imgLock) {
           angle =  targetAngle;
        }
    	return angle;
    }
    
    public double getTargetDistance() {
    	double distance = 99.9;
        synchronized (imgLock) {
        	distance =  targetDistance;
        }
    	return distance;
    }
    
    public boolean isTargetLocked() {
    	return isTargetLocked;
    }
    
    public AxisCamera getCamera() {
    	return camera;
    }
    
    
}

