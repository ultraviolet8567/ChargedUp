// feeds from gyro & vision
// calculates odometry data by taking weighted average from mentioned classes
// methods that output:
// robot translation (x, y, z)
// robot heading relative to the field

package frc.robot.odometry;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase {

  GyroOdometry gyro;
  VisionOdometry vision;

  private NetworkTableEntry camera1;

  // TODO: find the actual value
  private double framerate = 30;

  Optional<EstimatedRobotPose> pose;

  public Odometry(GyroOdometry gyro, VisionOdometry vision) {
    this.gyro = gyro;
    this.vision = vision;
  }

  @Override
  public void periodic() {
    // updates gyro
    gyro.updateGyroOdometry();

    // CCW+
    Logger.getInstance().recordOutput("Odometry/Heading", getRotation2d().getRadians());
  }

  // i have no idea what this does
  public double calculateVisionPercent() {
    double errorScale = Math.abs(gyro.getRate()) / Units.degreesToRadians(8.0);
    errorScale = MathUtil.clamp(errorScale, 0, 1); // errorscale is between 0 and 1
    double shift = 1 - Math.pow(1 - 0.85, 1 / framerate); // i have no idea why this is happening but it's happening!
    shift *= 1 - errorScale;

    return shift;
  }



  // average of X-values
  public double getX() {
    if (vision.checkCameras() >= 1) {
      return calculateAverage(vision.getX(), gyro.getX());
    } else {
      return gyro.getX();
    }
  }

  // average of Y-values
  public double getY() {
    if (vision.checkCameras() >= 1) {
      return calculateAverage(vision.getY(), gyro.getY());
    } else {
      return gyro.getY();
    }
  }

  // depth
  public double getZ() {
    if (vision.checkCameras() >= 1) {
      return vision.getZ();
    } else {
      return 0.0;
    }
  }

  // calculates heading (angle) average
  public Rotation3d getHeading() {
    if (vision.checkCameras() >= 1) {
      return calculateHeadingAverage(vision.getHeading(), gyro.getHeading());
    } else {
      return gyro.getHeading();
    }
  }

  // for logger
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(gyro.getReading());
  }

  // weighted average 
  public double calculateAverage(double visionValue, double gyroValue) {
    return (calculateVisionPercent() * visionValue) + ((100 - calculateVisionPercent()) * gyroValue);
  }

  // weight average for heading
  public Rotation3d calculateHeadingAverage(Rotation3d visionValue, Rotation3d gyroValue) {
    double roll = calculateAverage(visionValue.getX(), gyroValue.getX());
    double pitch = calculateAverage(visionValue.getY(), gyroValue.getY());
    double yaw = calculateAverage(visionValue.getZ(), gyroValue.getZ());

    return new Rotation3d(roll, pitch, yaw);
  }
}
