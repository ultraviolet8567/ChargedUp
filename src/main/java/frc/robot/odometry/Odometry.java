// feeds from gyro & vision
// calculates odometry data by taking weighted average from mentioned classes
// methods that output:
// robot translation (x, y, z)
// robot heading relative to the field

package frc.robot.odometry;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {

  Gyro gyro;
  Vision vision;

  Optional<EstimatedRobotPose> pose;

  public Odometry(Gyro gyro, Vision vision) {
    this.gyro = gyro;
    this.vision = vision;
    
  }

  @Override
  public void periodic() {

    // CCW+
    Logger.getInstance().recordOutput("Odometry/Heading", getRotation2d().getRadians());
  }

  public double calculateVisionPercent() {
    // stuff goes here
    return 4.0; // example value
  }


  public double getX() {
    return calculateAverage(vision.getX(), gyro.getX());
  }

  public double getY() {
    return calculateAverage(vision.getY(), gyro.getY());
  }

  public double getZ() {
    return vision.getZ();
  }

  public Rotation3d getHeading() {
    return calculateHeadingAverage(vision.getHeading(), gyro.getHeading());
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyro.getReading());
  }

  public double calculateAverage(double visionValue, double gyroValue) {
    return (calculateVisionPercent() * visionValue) + ((100 - calculateVisionPercent()) * gyroValue);
  }

  public Rotation3d calculateHeadingAverage(Rotation3d visionValue, Rotation3d gyroValue) {
    double roll = calculateAverage(visionValue.getX(), gyroValue.getX());
    double pitch = calculateAverage(visionValue.getY(), gyroValue.getY());
    double yaw = calculateAverage(visionValue.getZ(), gyroValue.getZ());

    return new Rotation3d(roll, pitch, yaw);
  }

  /** what Jonah wrote (for reference, do NOT use this)
   *  public void addVisionMeasurement(TimestampedTranslation2d data) {
    Optional<Pose2d> historicalFieldToTarget = poseHistory.get(data.timestamp);
    if (historicalFieldToTarget.isPresent()) {

      // Calculate new robot pose
      Rotation2d robotRotation = historicalFieldToTarget.get().getRotation();
      Rotation2d cameraRotation =
          robotRotation.rotateBy(vehicleToCamera.getRotation());
      Transform2d fieldToTargetRotated =
          new Transform2d(FieldConstants.hubCenter, cameraRotation);
      Transform2d fieldToCamera = fieldToTargetRotated.plus(
          GeomUtil.transformFromTranslation(data.translation.unaryMinus()));
      Pose2d visionFieldToTarget = GeomUtil
          .transformToPose(fieldToCamera.plus(vehicleToCamera.inverse()));

      // Save vision pose for logging
      noVisionTimer.reset();
      lastVisionPose = visionFieldToTarget;

      // Calculate vision percent
      double angularErrorScale =
          Math.abs(inputs.gyroVelocityRadPerSec) / visionMaxAngularVelocity;
      angularErrorScale = MathUtil.clamp(angularErrorScale, 0, 1);
      double visionShift =
          1 - Math.pow(1 - visionShiftPerSec, 1 / visionNominalFramerate);
      visionShift *= 1 - angularErrorScale;

      // Reset pose
      Pose2d currentFieldToTarget = getPose();
      Translation2d fieldToVisionField = new Translation2d(
          visionFieldToTarget.getX() - historicalFieldToTarget.get().getX(),
          visionFieldToTarget.getY() - historicalFieldToTarget.get().getY());
      Pose2d visionLatencyCompFieldToTarget =
          new Pose2d(currentFieldToTarget.getX() + fieldToVisionField.getX(),
              currentFieldToTarget.getY() + fieldToVisionField.getY(),
              currentFieldToTarget.getRotation());

      if (resetOnVision) {
        setPose(new Pose2d(visionFieldToTarget.getX(),
            visionFieldToTarget.getY(), currentFieldToTarget.getRotation()),
            true);
        resetOnVision = false;
      } else {
        setPose(new Pose2d(
            currentFieldToTarget.getX() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getX() * visionShift,
            currentFieldToTarget.getY() * (1 - visionShift)
                + visionLatencyCompFieldToTarget.getY() * visionShift,
            currentFieldToTarget.getRotation()), false);
      }
    }
  }
   */

}
