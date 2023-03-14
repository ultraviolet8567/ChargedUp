package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PhotonVision extends SubsystemBase {

  public static PhotonPoseEstimator estimator;

  private PhotonCamera camera1;

  public PhotonVision() {
    camera1 = new PhotonCamera("camera1");
    estimator = new PhotonPoseEstimator(Constants.FieldConstants.aprilTags, PoseStrategy.LOWEST_AMBIGUITY, camera1, Constants.CameraConstants.robotToCamera1);
  }

  public double distanceTag(PhotonPipelineResult result) {
    double distance;

    if (result.hasTargets()) { // checks that there are targets in the given result
      double range = PhotonUtils.calculateDistanceToTargetMeters(
        Constants.CameraConstants.camera1Displacement.getZ(), 
        Constants.tagHeight, 
        Constants.CameraConstants.camera1Direction.getY(), 
        Units.degreesToRadians(result.getBestTarget().getPitch()));
      distance = range;
    } else {
      distance = 0.0;
    }

    return distance;
  }

  // get the "best" (most reliable) tag's ID
  public Integer getTagID() {
    PhotonPipelineResult result = camera1.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return target.getFiducialId();
  }

  // updates pose estimator, used for odometry updates
  public static Optional<EstimatedRobotPose> getEstimate(Pose2d previousEstimate) {
    // estimator.setReferencePose(previousEstimate); // only used when using PoseStrategy.CLOSEST_TO_REFERENCE_POSE
    return estimator.update(); // creates estimated robot poses, used to optimize odometry
  }
}
