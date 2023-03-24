package frc.robot.odometry;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.geometry.Rotation3d;

public class VisionOdometry extends SubsystemBase {
  // TODO: check if the IDs are correct at all (or just find them)
  // also, connect the camera to NetworkTables so that they work?
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
  // public ArrayList<Optional<EstimatedRobotPose>> updateVisionOdometry() {
  //   ArrayList<Optional<EstimatedRobotPose>> results = new ArrayList<>();
  //   for (int i = 0; i <= 2; i++) {
  //       try {
  //           Thread.sleep(20);
  //           for (Camera camera : cameras) {
  //               if (camera.checkDetections()) {
  //                   results.add(camera.getEstimate());
  //               } else {
  //                   results.add(null);
  //               }
  //           }
  //       } catch (InterruptedException e) {
  //       }
  //   }

  //   return results;
  // }

  // gets X-translational values from cameras that have detections and averages them
  public double getX() {
    double x = 0.0;
    if (checkCameras() >= 1) {
      for (Camera camera : cameras) {
          x += camera.getX();
      }
    }
    int j = checkCameras();
    if (j != 0)
      return x / j;
    else
      return x;
  }

    // gets Y-translational values from cameras that have detections and averages them
  public double getY() {
    double y = 0.0;
    if (checkCameras() >= 1) {
      for (Camera camera : cameras) {
        y += camera.getY();
      }
    }
    int j = checkCameras();
    if (j != 0)
      return y / j;
    else
      return y;
  }

    // gets Z-translational values (depth) from cameras that have detections and averages them
  public double getZ() {
    double z = 0.0;
    if (checkCameras() >= 1) {
      for (Camera camera : cameras) {
          z += camera.getZ();
      }
    }
    int j = checkCameras();
    if (j != 0)
      return z / j;
    else
      return z;
  }

// gets rotational vlues from cameras that have detections and averages them
  public Rotation3d getHeading() {
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    if (checkCameras() >= 1) {
      for (Camera camera : cameras) {
        roll += camera.getX();
        pitch += camera.getY();
        yaw += camera.getZ();
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

  public int getTag() {
    return camera1.getTag();
  }
}