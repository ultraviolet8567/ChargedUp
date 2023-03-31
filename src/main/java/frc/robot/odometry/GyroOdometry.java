package frc.robot.odometry;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class GyroOdometry extends SubsystemBase {
    private Swerve swerve;
    private AHRS gyro;
    private SwerveDriveOdometry odometer;

    // TODO: The initial position estimate of the robot; may vary match to match
    private static final Pose2d initialPose = new Pose2d(1.86, 0, new Rotation2d(0));
    
    // private double latestXPose;
    // Minimum angle for slope in radians
    // private static final double minimumAngle = Math.PI / 6; 

    public GyroOdometry(Swerve swerve) {
        this.swerve = swerve;
        gyro = new AHRS(SPI.Port.kMXP);
        odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, new Rotation2d(), swerve.getModulePositions(), initialPose);
        
        // new Thread(() -> {
        //     try {
        //         Thread.sleep(100);
        //         gyro.calibrate();
        //     } catch (Exception e) { }
        // });
    }

    public void periodic() {
        Logger.getInstance().recordOutput("Odometry/Pose", getPose());
        Logger.getInstance().recordOutput("Odometry/PoseInverted", invertPose(getPose()));
        Logger.getInstance().recordOutput("Odometry/Heading", Math.IEEEremainder(getRotation2d().getRadians(), 2 * Math.PI));
        Logger.getInstance().recordOutput("Odometry/Rotation3d/Roll", getRotation3d().getX());
        Logger.getInstance().recordOutput("Odometry/Rotation3d/Pitch", getRotation3d().getY());
        Logger.getInstance().recordOutput("Odometry/Rotation3d/Yaw", getRotation3d().getZ());
        
        odometer.update(getRotation2d(), swerve.getModulePositions());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public Pose2d invertPose(Pose2d original) {
        return new Pose2d(-original.getX(), -original.getY(), original.getRotation());
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), swerve.getModulePositions(), pose);
    }
    
    // Returns angular values in Rrotation3d format
    public Rotation3d getRotation3d() {
        return new Rotation3d(
            // Negate the reading because the navX has CCW- and we need CCW+
            Rotation2d.fromDegrees(-gyro.getRoll()).getRadians(),
            Rotation2d.fromDegrees(-gyro.getPitch()).getRadians(),
            Rotation2d.fromDegrees(-gyro.getYaw()).getRadians());
    } 

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() {
        // Negate the reading because the navX has CCW- and we need CCW+
        double reading = -gyro.getAngle();
        // Calculate this offset for the field when at pit setup
        // reading += Constants.kGyroOffset;

        // Add π radians since 0 is the opposite for blue
        // if (DriverStation.getAlliance() == Alliance.Blue)
        //     reading += Math.PI;

        // if (Constants.kDirectionSwitch)
        //     reading += Math.PI;
            
        return reading;
    }

    // On pit setup day, take robot to corner of field and reset (set 0, 0)
    public void resetGyro() {
        gyro.zeroYaw();
        resetOdometry(initialPose);
    }

    public double getRate() {
        return gyro.getRate();
    }

    
    // public void updateGyroOdometry() {
    //     // Check if on a slope
    //     if (Math.abs(getRotation3d().getX()) > minimumAngle || Math.abs(getRotation3d().getY()) > minimumAngle) {
    //         slopeCalc();
    //     }
    //     else {
    //         estimator.update(getRotation2d(), swerve.getModulePositions());
    //     }

    //     latestXPose = getPose2d().getX();
    // }

    // public void slopeCalc() {
    //     estimator.update(getRotation2d(), swerve.getModulePositions());
    //     double pdX = getPose2d().getX() - latestXPose;
    //     double adX = pdX * Math.cos(getHeading());
    //     double aCurrentX = latestXPose + adX;
    //     Pose2d updatedPose = new Pose2d(new Translation2d(aCurrentX, getHeading()), getRotation2d());

    //     // Hope and pray that this sets the current position to the updatedPose and doesn't just break things
    //     estimator.resetPosition(gyro.getRotation2d(), swerve.getModulePositions(), updatedPose);
    // }
}