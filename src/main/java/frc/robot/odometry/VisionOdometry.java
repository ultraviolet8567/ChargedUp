package frc.robot.odometry;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Rotation3d;

public class VisionOdometry {
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
    for (int i = 0; i <= 2; i++) {
        try {
            Thread.sleep(20);
            for (Camera camera : cameras) {
                if (camera.checkDetections()) {
                    results.add(camera.getEstimate());
                } else {
                    results.add(null);
                }
            }
        } catch (InterruptedException e) {

        }
    }

    return results;
  }

  // gets X-translational values from cameras that have detections and averages them
  public double getX(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double x = 0.0;

    for (Optional<EstimatedRobotPose> detection : detections) {
      if (detection.isPresent()) {
        x += detection.get().estimatedPose.getX();
      }
    }

    return x / checkCameras();
  }

    // gets Y-translational values from cameras that have detections and averages them
  public double getY(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double y = 0.0;

    for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection.isPresent()) {
          y += detection.get().estimatedPose.getY();
        }
    }

    return (y / checkCameras());
  }

    // gets Z-translational values (depth) from cameras that have detections and averages them
  public double getZ(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double z = 0.0;

    for (Optional<EstimatedRobotPose> detection : detections) {
        if (detection.isPresent()) {
          z += detection.get().estimatedPose.getZ();
        }
    }

    return (z / checkCameras());
  }

// gets rotational vlues from cameras that have detections and averages them
  public Rotation3d getHeading(ArrayList<Optional<EstimatedRobotPose>> detections) {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    for (Optional<EstimatedRobotPose> detection : detections) {
      if (detection.isPresent()) {
        roll += detection.get().estimatedPose.getRotation().getX();
        pitch += detection.get().estimatedPose.getRotation().getX();
        yaw += detection.get().estimatedPose.getRotation().getX();
      }
    }

    return new Rotation3d(
      (roll / checkCameras()), 
      (pitch / checkCameras()), 
      (yaw / checkCameras()));
  }
}
