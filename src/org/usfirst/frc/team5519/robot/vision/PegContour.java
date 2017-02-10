package org.usfirst.frc.team5519.robot.vision;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

/**
 * 
 * @author tenso
 * 
 * This class defines an identified contour that is one half of a peg target.
 * In the Steamworks game, this contour is a vertical rectangle of retro-reflective tape
 *  that is 5 inches high and 2 inches wide located 10.75 inches above the floor.
 *
 */
public class PegContour {
	
	static double inchesHeight = 5.0;
	static double inchesWidth = 2.0;
	static double aspectRatio = inchesHeight / inchesWidth;
	
	private MatOfPoint myContour;
	private Rect boundingRectangle;
	
	/**
	 * Checks contour against expectations for a peg visual target. 
	 * Must be a vertical rectangle with aspect ratio of about 2.5
	 * @param contour
	 * @return True if contour matches peg target requirements
	 */
	static boolean isaPegContour(MatOfPoint contour) {
		Rect r = Imgproc.boundingRect(contour);
		if (r.height > r.width) {   // Must be vertical rectangle
			double aspect = ((double) r.height) / ((double) r.width);
			if ((aspect >=1.5) & (aspect <= 3.5)) { // Must have an aspect ratio of about 2.5
				// Could additionally check for expected height of 10.75 inches above the floor
				return true;
			}
		}
		return false;
	}
	
	public PegContour(MatOfPoint contour) {
		myContour = contour;
		boundingRectangle = Imgproc.boundingRect(contour);
	}
	
	public MatOfPoint getContour() {
		return myContour;
	}
	
	public Rect getBoundingRectangle() {
		return boundingRectangle;
	}
	
	public int getArea() {
		return (boundingRectangle.height * boundingRectangle.width);
	}
	
	public int getLeftEdge() {
		return boundingRectangle.x;
	}

}
