// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// =============== ============ UNSUPPORTED ============ =============== // 
// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import edu.wpi.first.apriltag.*;
// import edu.wpi.first.apriltag.AprilTagDetector.Config;
// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.UsbCameraInfo;
// import edu.wpi.first.cscore.VideoMode.PixelFormat;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.Timer;

// import org.opencv.core.Core;
// import org.opencv.core.Mat;
// import org.opencv.core.Point;
// import org.opencv.core.Scalar;
// import org.opencv.imgproc.Imgproc;

// import frc.robot.RobotContainer;

// import java.util.ArrayList;
// import java.util.List;

// public class Vision extends SubsystemBase {
//   // camera (realsense D435)
//   // max RGB resolution: 1920 x 1080
//   // frame rate: 30 fps
//   // has a depth camera as well, exclude that
//   UsbCamera camera;

//   CvSource imageStream;
//   CvSource maskStream;

//   // TODO: Tweak values below to make apriltag detection consistent
//   double lowSaturation = 0;
//   double highSaturation = 12; // (0 - 255) adjust for high saturation value: the higher this value the greater the deviation from
//                               // grayscale that will still be accepted 
//                               // TODO: saturation filter currently disabled - optimize and check for re-enabling
//   double minArea = 75; // (sq. pixels) minimum area of a detection that will be kept and considered
//   double minSideLength = 64; // (lin. pixels) minimum length of any of the sides of the quad created by a detection

//   // TODO: stream to dashboard
//   // smartdashboard
//   SmartDashboard dashboard;

//   // source for video, provides sequence of frames
//   // CvSink is specialized for OpenCV - accepts video frames as OpenCV images
//   CvSink sink;

//   // AprilTag detector engine
//   AprilTagDetector detector = new AprilTagDetector();

//   AprilTagDetector.QuadThresholdParameters params = new AprilTagDetector.QuadThresholdParameters(); // lies, fairy dust, deceit


//   // NOT NEEDED FOR NOW
//   // configuration for below
//   AprilTagPoseEstimator.Config posConfig = new AprilTagPoseEstimator.Config(Constants.tagSize, Constants.fx, Constants.fy, Constants.cx, Constants.cy);
//   // predicts and tracks location of tag

//   // creates the timer for lowering capture rate
//   Timer captureTimer = new Timer();

//   // mat is an OpenCV data type that represents an n-dimensional array
//   // made of a header (size, storing method, etc.) and a pointer (pixel values of image)
//   Mat mat; // RGB
//   Mat graymat; // grayscale
//   Mat processed; // grayscale & processed

//   // an array of camera information (includes, probably, the RGB realsense, depth realsense, and webcam)
//   UsbCameraInfo[] info;

//   // size of one side of an april-tag detection
//   double hSideLength;
//   double vSideLength;
  
//   // list of sidelengths of detection quads
//   List<Double> QuadSideLength = new ArrayList<Double>();

//   public Vision() {
//     // sets up USB camera capture
//     // TODO: for every new laptop, refind the value (uncomment the stuff below this line)
//     // Alexis: 0
//     // Sid: 0
//     // Chaerin: 1
//     // rio: 3

//     camera = CameraServer.startAutomaticCapture("Color", "/dev/video2");

//     camera.setPixelFormat(PixelFormat.kYUYV);
//     camera.setFPS(30);
//     camera.setResolution(424, 240);

//     imageStream = CameraServer.putVideo("Image Stream", 424, 240); 
//     maskStream = CameraServer.putVideo("Masked Area", 424, 240);

//     //shuffleboard stuff
//     RobotContainer.cameraTab.add("Camera", camera).withWidget(BuiltInWidgets.kCameraStream)
//       .withSize(3, 2)
//       .withPosition(0, 1)
//     ;

//     RobotContainer.cameraTab.add("Processed Image", imageStream).withWidget(BuiltInWidgets.kCameraStream)
//       .withSize(3, 2)
//       .withPosition(0, 2)
//     ;

//     // how many cameras do we have?
//     // info = camera.enumerateUsbCameras();

//     // int i = 0;
//     // for (UsbCameraInfo camera : info) {
//     //   System.out.println(i + ": " + camera.name + ": " + camera.path); // name and path
//     //   i += 1;
//     // }

//     // get OpenCV access to primary camera feed - can get images from camera for processing on RIO
//     sink = CameraServer.getVideo();

//     // which family of tags are we detecting?
//     detector.addFamily("tag16h5");
    
//     detector.setQuadThresholdParameters(params);

//     // making mats
//     mat = new Mat(); // RGB
//     graymat = new Mat(); // grayscale

//     System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

//     //sets up AprilTagPoseEstimator
//     AprilTagPoseEstimator estimator = new AprilTagPoseEstimator(posConfig);

//     //sets up the timer for lowering capture rates
//     captureTimer.start();

//     // did we get here?
//     // System.out.println("i got here");

//   }

//   @Override
//   public void periodic() {
//     // this code hasn't been interrupted, has it?
//     while (!Thread.interrupted()) {

//       // set in-between run time to be 1 second
//       // if (captureTimer.get() > 1.0) {
//       //   captureTimer.reset();


//         // grabs image from camera
//         long time = sink.grabFrame(mat);

//         // has there been an error of some kind?
//         if (time == 0) {
//           // don't stop (never mind that)
//           continue;
//         }
        
//         // converts image to grayscale (mat -> graymat)
//         Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_BGR2GRAY);

//         // list of all detected tags' ids
//         List<Integer> ids = new ArrayList<Integer>();

//         // we don't have any ids yet
//         ids.clear();

//         // detect tag
//         // TODO: why did the camera get these when there were no apriltags?
//         // ANSWER??: the webcam was on
//         for (AprilTagDetection detection : detector.detect(graymat)) {
//           // run estimator
//           // Transform3d pose = estimator.estimate(detection);
          
//           double Area = 0.0; // haven't calculated area of detection yet
//           QuadSideLength.clear(); // clear quadsidelength of previous quad
          
//           for (var i = 0; i <= 3; i++) { // get side lengths of each side of this quad and the area of this quad
//             var j = (i + 1) % 4;

//             Area += detection.getCornerX(i) * detection.getCornerY(j); // shoelace formula for area
//             Area -= detection.getCornerX(j) * detection.getCornerY(i);

//             hSideLength = Math.abs(detection.getCornerX(i) - detection.getCornerX(j)); // horizontal size of line
//             vSideLength = Math.abs(detection.getCornerY(i) - detection.getCornerY(j)); // vertical size of line
            
//             QuadSideLength.add(Math.sqrt(Math.pow(hSideLength, 2) + Math.pow(vSideLength, 2))); // find and add the length of the line to 
//                                                                                                 // the array
          
//           }

//           Boolean shouldContinue = false;
//           for (int i = 0; i <= 3; i++) {
//             if (QuadSideLength.get(i) < minSideLength)
//               shouldContinue = true; // checks if any of the side lengths are smaller than the minimum, and 
//                                      // skips logging and processing this detection if they are
//                                      // TODO: check for side length ratio OR understand homography matrices to filter (slr better)
//           }

//           if (shouldContinue) {
//             continue; // incredibly complex piece of code. let me take you through it:
//                       // if you should Continue: you continue.
//           }


          
//           Area /= 2.0;
//           Area = Math.abs(Area); // processing Area variable to be positive and the correct magnitude

//           if (Area < minArea)
//             continue; // if Area is less than minimum area, skip logging and processing this detection


//           for (var i = 0; i <= 3; i++) { // drawing lines around detection
//             var j = (i + 1) % 4;

//             var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
//             var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
            
//             Imgproc.line(mat, pt1, pt2, new Scalar(255, 0, 255), 2);
//           }
          
//           // adds this tag's id to list
//           ids.add(detection.getId());
//         // }

//         // prints list of tag ids
//         // adds the frame with detections highlighted to the output stream
//       }

//       System.out.println(ids);

//       imageStream.putFrame(mat);

//     }
//   }

//   public void processImage() { // TODO: image processing required?

//     // option 0: do nothing to the grayscale image.
//     // Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_BGR2GRAY); 
//     // processed.setTo(graymat);

//     // option 1: bilateral filter, supposed to remove noise and blur while keeping edges, but expensive
//     //Imgproc.bilateralFilter(graymat, processed, 5, 10, 10); //i don't know what these values mean
    
//     // option 2: just regular old blur. might wreak havoc on the edges
//     // Size size = new Size(3, 3); // 3x3 filter
//     // Imgproc.blur(graymat, processed, size);
    
//     // option 3: isolate only low-saturated (whitish, blackish, greyish) colors and then greyscale
//     // Scalar nearBlack = new Scalar(0, lowSaturation, 0); // pure black
//     // Scalar nearWhite = new Scalar(179, highSaturation, 255); // bluish-white. 
//     // // note on above range: it checks for any hue (0-179), a saturation below 20 (0-20), and any brightness (0-255)
//     // Imgproc.cvtColor(mat, processed, Imgproc.COLOR_BGR2HSV); // processed is original but in HSV
//     // Core.inRange(processed, nearBlack, nearWhite, graymat); // graymat becomes the mask for a bit
//     // Core.bitwise_and(mat, mat, processed, graymat); // processed is set to mask region but only in BGR color
//     // // graymat.setTo(processed); //set graymat to processed, graymat is now mask region but in BGR color
//     // Imgproc.cvtColor(processed, graymat, Imgproc.COLOR_BGR2GRAY); // final output, graymat is is only the masked region, in grayscale

//   }

//   // TODO: apply this after refining detection to be consistent

//   // public Pose3d detectFromImage(String path) {
    
//   //   Pose3d output;

//   //   // grab image from path
//   //   Mat snapshot = readImage(path);
//   //   // run AprilTagDetector on the image
//   //   AprilTagDetection[] detection = detector.detect(snapshot);
//   //   // run AprilTagPoseEstimator on the detection
//   //   Transform3d tagPose = estimator.estimate(detection[0]);
//   //   // get the tagID from the detection
//   //   int tagID = detection[0].getId();
//   //   // use the tagID to get the tag position from constants, then subtract the tagPose from it to
//   //   // get the robot's position
//   //   output = Constants.FieldConstants.aprilTags.get(tagID).plus(tagPose.inverse());

//   //   return output;
//   // }
// }
