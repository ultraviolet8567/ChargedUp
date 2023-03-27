package frc.robot.odometry;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
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

    public Pose3d getEstimatedPose() {
        return getEstimate().get().estimatedPose;
    }

    public boolean checkDetections() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (!result.hasTargets())
            return false;
        else
            return (result.getBestTarget().getPoseAmbiguity() >= 0.0 && result.getBestTarget().getPoseAmbiguity() < 0.15);
    }

    public int getTag() {
        return camera.getLatestResult().getBestTarget().getFiducialId();
    }
}