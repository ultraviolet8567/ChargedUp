// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.SPI;

public class Gyro {

  Swerve swerve;

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  SwerveModulePosition[] modulePositions = swerve.getModulePositions();

  // the pose2d is the starting pose estimate of the robot (find, position)
  public SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), modulePositions, new Pose2d());

  public Gyro(Swerve swerve) {
    this.swerve = swerve;
    new Thread(() -> {
      try {
          Thread.sleep(1000);
          gyro.calibrate();
          resetGyro();
      } catch (Exception e) {

      }
    });
  }

  // on pit setup day, take robot to corner of field and record as 0
  public void resetGyro() {
      gyro.reset();
  }

  public void updateGyroOdometry() {
    estimator.update(getRotation2d(), swerve.getModulePositions());
  }

  public Rotation3d getHeading() {
    return new Rotation3d(gyro.getRoll(), gyro.getPitch(), gyro.getYaw());
  } 

  public double getReading() {
    // Negate the reading because the navX has CCW- and we need CCW+
    return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getReading());
  }

  public double getX() {
    return estimator.getEstimatedPosition().getX();
  }

  public double getY() {
    return estimator.getEstimatedPosition().getY();
  }
}
