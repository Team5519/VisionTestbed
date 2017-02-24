package org.usfirst.frc.team5519.robot.subsystems;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 *
 */
public class ShooterCamera extends Subsystem {
	
	private static int reticlePositionX = 160;		// Midpoint 320x240
	private static Scalar reticleColour = new Scalar(0,0,0);
	private static int reticleThickness = 2;
	private static int reticleLineType = 8;
	
	private UsbCamera camera;
	private Mat reticle;

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    private void drawLongLine(int yPos) {
    	Point point1 = new Point(reticlePositionX-8,yPos);
    	Point point2 = new Point(reticlePositionX+8,yPos);
    	Imgproc.line(reticle, point1, point2, reticleColour, reticleThickness, reticleLineType,0);   	
    }
    
    private void drawShortLine(int yPos) {
    	Point point1 = new Point(reticlePositionX-5,yPos);
    	Point point2 = new Point(reticlePositionX+5,yPos);
    	Imgproc.line(reticle, point1, point2, reticleColour, reticleThickness, reticleLineType,0);   	
    }
    
    private void drawMidLine(int yPos) {
    	Point point1 = new Point(reticlePositionX-10,yPos);
    	Point point2 = new Point(reticlePositionX+10,yPos);
    	Imgproc.line(reticle, point1, point2, reticleColour, reticleThickness, reticleLineType,0);   	
    }
    
    private void drawReticle() {
    	drawLongLine(20);
    	drawShortLine(45);
    	drawLongLine(70);
    	drawShortLine(95);
    	drawMidLine(120);
    	drawShortLine(145);
    	drawLongLine(170);
    	drawShortLine(195);
    	drawLongLine(220);
     }
    
    public ShooterCamera() {
    	camera = CameraServer.getInstance().startAutomaticCapture();
        Mat snapshot = new Mat();
        CvSink cvSink = CameraServer.getInstance().getVideo(camera);
    }

}

