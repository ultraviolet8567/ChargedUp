package frc.robot.odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

public class Camera {

  public static PhotonPoseEstimator estimator;

  private PhotonCamera camera;

  private int ID;

  public Camera(int cameraID) {
    ID = cameraID;

    camera = new PhotonCamera("camera" + cameraID);

    estimator = new PhotonPoseEstimator(Constants.FieldConstants.aprilTags, PoseStrategy.LOWEST_AMBIGUITY, camera, Constants.Camera.cameraDistances[ID]);
  }

  // updates pose estimator, used for odometry updates
  public Optional<EstimatedRobotPose> getEstimate() {
    return estimator.update(); // creates estimated robot poses
  }

  public double getX() {
    return getEstimate().get().estimatedPose.getX();
  }

  public double getY() {
    return getEstimate().get().estimatedPose.getY();
  }

  public double getZ() {
    return getEstimate().get().estimatedPose.getZ();
  }

  public Rotation3d getHeading() {
    return getEstimate().get().estimatedPose.getRotation();
  }

  public boolean checkDetections() {
    PhotonPipelineResult result = camera.getLatestResult();
    
    if (!result.hasTargets())
      return false;
    else {
      if (result.getBestTarget().getPoseAmbiguity() >= 0.0 && result.getBestTarget().getPoseAmbiguity() < 0.15)
        return true;
      else return false;
    }
  }
}
