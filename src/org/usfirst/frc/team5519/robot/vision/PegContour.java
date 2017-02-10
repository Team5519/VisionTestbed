package org.usfirst.frc.team5519.robot.vision;

public class PegContour {


}

// Detect faces in the image.
// MatOfRect is a special container class for Rect.
MatOfRect faceDetections = new MatOfRect();
faceDetector.detectMultiScale(image, faceDetections);

System.out.println(String.format("Detected %s faces", faceDetections.toArray().length));

// Draw a bounding box around each face.
for (Rect rect : faceDetections.toArray()) {
    Core.rectangle(image, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height), new Scalar(0, 255, 0));
}


