package org.usfirst.frc.team5519.robot.subsystems;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;


/**
 *
 */
public class AxisVision extends Subsystem {

	private static final int IMG_WIDTH = 320;
	private static final int IMG_HEIGHT = 240;
	
	private AxisCamera camera;
	private boolean inVisionMode;
	private VisionThread visionThread;
	
	private final Object imgLock = new Object();
	private double centerX = 0.0;

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
    	
        new Thread(() -> {
            //AxisCamera camera = CameraServer.getInstance().addAxisCamera("Axis Stream","axis-camera");
            //camera.setResolution(640, 480);
            
            CvSink cvSink = CameraServer.getInstance().getVideo();
            CvSource outputStream = CameraServer.getInstance().putVideo("Peg Vision", IMG_WIDTH, IMG_HEIGHT);
            
            Mat source = new Mat();
            Mat output = new Mat();
            /**
            PegVisionPipeline pipeline = new PegVisionPipeline();
            
            while(!Thread.interrupted()) {
                cvSink.grabFrame(source);
                //Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                pipeline.process(source);
                output = pipeline.hslThresholdOutput();
                outputStream.putFrame(output);
                Timer.delay(0.1);
            }
            */
            
        }).start();

    	
        /**
        visionThread = new VisionThread(camera, new PegVisionPipeline(), pipeline -> {
            if (!pipeline.filterContoursOutput().isEmpty()) {
                Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
                synchronized (imgLock) {
                    centerX = r.x + (r.width / 2);
                }
            }
        });
        visionThread.start();
        */

    }
    
    public AxisCamera getCamera() {
    	return camera;
    }
    
    
}

