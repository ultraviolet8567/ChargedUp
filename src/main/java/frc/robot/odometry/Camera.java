package frc.robot.odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Camera {

  public static PhotonPoseEstimator estimator;

  private PhotonCamera camera;

  private Integer ID;

  public Camera(Integer cameraID) {
    ID = cameraID;

    camera = new PhotonCamera("camera" + cameraID);

    estimator = new PhotonPoseEstimator(Constants.FieldConstants.aprilTags, PoseStrategy.LOWEST_AMBIGUITY, camera, Constants.CameraConstants.cameraDistances[ID]);
  }

  // find distance from tag to robot
  public double distanceTag(PhotonPipelineResult result) {
    double distance;

    if (result.hasTargets()) { // checks that there are targets in the given result
      double range = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.CameraConstants.cameraDisplacements[ID].getZ(), 
        Constants.tagHeight, 
        Constants.CameraConstants.cameraDirections[ID].getY(), 
        Units.degreesToRadians(result.getBestTarget().getPitch()));
      distance = range;
    } else {
      distance = 0.0;
    }

    return distance;
  }

  // get the "best" (most reliable) tag's ID
  public Integer getTagID() {
    PhotonPipelineResult result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getFiducialId();
  }

  // updates pose estimator, used for odometry updates
  public static Optional<EstimatedRobotPose> getEstimate() {
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
    return result.hasTargets();
  }

}
