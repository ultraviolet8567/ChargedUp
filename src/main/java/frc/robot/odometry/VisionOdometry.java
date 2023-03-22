package frc.robot.odometry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Rotation3d;

public class VisionOdometry extends SubsystemBase {
  private final Camera camera1 = new Camera(0);
  private final Camera camera2 = new Camera(1);
  private final Camera camera3 = new Camera(2);

  private final Camera[] cameras = new Camera[] {
    camera1, 
    camera2,  
    camera3
  };

  public VisionOdometry() {}

  // counts the number of cameras that have detected something
  public int checkCameras() {
    int count = 0;

    for (Camera camera : cameras) {
      if (camera.checkDetections()) {
        count += 1;
      }
    }

    return count;
  }

  // list of detections that were detected in a 60ms time frame
  public ArrayList<Optional<EstimatedRobotPose>> updateVisionOdometry() {
    
    ArrayList<Optional<EstimatedRobotPose>> results = new ArrayList<>();
    
    for (Camera camera : cameras) {
      if (camera.checkDetections()) {
        results.add(camera.getEstimate());
      } else {
        results.add(null);
      }
    }

    return results;
  }

  // gets X-translational values from cameras that have detections and averages them
  public double getX(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double x = 0.0;
    if (!detections.isEmpty()) {
      for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection != null) {
          x += detection.get().estimatedPose.getX();
        }
      }
    }
    int j = checkCameras();
    if (j != 0)
      return x / j;
    else
      return x;
  }

    // gets Y-translational values from cameras that have detections and averages them
  public double getY(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double y = 0.0;
    if (!detections.isEmpty()) {
      for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection != null) {
          y += detection.get().estimatedPose.getY();
        }
      }
    }
    int j = checkCameras();
    if (j != 0)
      return y / j;
    else
      return y;
  }

    // gets Z-translational values (depth) from cameras that have detections and averages them
  public double getZ(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double z = 0.0;
    if (!detections.isEmpty()) {
      for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection != null) {
          z += detection.get().estimatedPose.getZ();
        }
      }
    }
    int j = checkCameras();
    if (j != 0)
      return z / j;
    else
      return z;
  }

// gets rotational vlues from cameras that have detections and averages them
  public Rotation3d getHeading(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    if (!detections.isEmpty()) {
      for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection != null) {
          roll += detection.get().estimatedPose.getRotation().getX();
          pitch += detection.get().estimatedPose.getRotation().getY();
          yaw += detection.get().estimatedPose.getRotation().getZ();
        }
      }
    }
    int j = checkCameras();
    if (j != 0) {
      return new Rotation3d(
        roll / j, 
        pitch / j, 
        yaw / j);
    }
    else {
      return new Rotation3d(
        roll, 
        pitch, 
        yaw);
    }
  }
}
