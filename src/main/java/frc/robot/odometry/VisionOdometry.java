package frc.robot.odometry;

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
  // causes loop overrun errors :(
//   public ArrayList<Optional<EstimatedRobotPose>> updateVisionOdometry() {
//     ArrayList<Optional<EstimatedRobotPose>> results = new ArrayList<>();
//     for (int i = 0; i <= 2; i++) {
//         try {
//             Thread.sleep(20);
//             for (Camera camera : cameras) {
//                 if (camera.checkDetections()) {
//                     results.add(camera.getEstimate());
//                 } else {
//                     results.add(null);
//                 }
//             }
//         } catch (InterruptedException e) {

//         }
//     }

//     return results;
//   }

  // gets X-translational values from cameras that have detections and averages them
  public double getX() {
    double x = 0.0;

    for (Camera camera : cameras) {
        if (camera.checkDetections()) {
          x += camera.getX();
        }
    }

    return x / checkCameras();
  }

    // gets Y-translational values from cameras that have detections and averages them
  public double getY() {
    double y = 0.0;

    for (Camera camera : cameras) {
        if (camera.checkDetections()) {
          y += camera.getY();
        }
    }

    return (y / checkCameras());
  }

    // gets Z-translational values (depth) from cameras that have detections and averages them
  public double getZ() {
    double z = 0.0;

    for (Camera camera : cameras) {
        if (camera.checkDetections()) {
          z += camera.getZ();
        }
    }

    return (z / checkCameras());
  }

// gets rotational vlues from cameras that have detections and averages them
  public Rotation3d getHeading() {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    for (Camera camera : cameras) {
        if (camera.checkDetections()) {
          roll += camera.getHeading().getX();
          pitch += camera.getHeading().getY();
          yaw += camera.getHeading().getZ();
        }
    }

    return new Rotation3d(
      (roll / checkCameras()), 
      (pitch / checkCameras()), 
      (yaw / checkCameras()));
  }
}
