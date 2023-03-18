package frc.robot.odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class GyroOdometry {

  Swerve swerve;

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  SwerveModulePosition[] modulePositions = swerve.getModulePositions();

  // the pose2d is the starting pose estimate of the robot (find, position)
  public SwerveDrivePoseEstimator estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), modulePositions, new Pose2d());

  public GyroOdometry(Swerve swerve) {
    this.swerve = swerve;
    new Thread(() -> {
      try {
          Thread.sleep(100);
          gyro.calibrate();
          resetGyro();
      } catch (Exception e) { }
    });
  }

  // on pit setup day, take robot to corner of field and record as 0
  public void resetGyro() {
      gyro.reset();
  }

  // updates the pose estimator
  public void updateGyroOdometry() {
    estimator.update(getRotation2d(), swerve.getModulePositions());
  }

  // returns angular(?) values in rotation3d format
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

  // gets X-translational value
  public double getX() {
    return estimator.getEstimatedPosition().getX();
  }

  // gets Y-translational value
  public double getY() {
    return estimator.getEstimatedPosition().getY();
  }

  public double getRate() {
    return gyro.getRate();
  }
}
