package frc.robot.odometry;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class GyroOdometry extends SubsystemBase {
    Swerve swerve;
    private AHRS gyro = new AHRS(SPI.Port.kMXP);
    SwerveModulePosition[] modulePositions;
  
    boolean onSlope;
    private double lastX;
    // private double slopeY;
    
    private static double minimumAngle = 0.03; // minimum angle for slope in radians

    // the pose2d is the starting pose estimate of the robot 
    // TODO: (find initial position, may vary based on match)
    public SwerveDrivePoseEstimator estimator;

    public GyroOdometry(Swerve swerve) {
        this.swerve = swerve;
        modulePositions = swerve.getModulePositions();
        onSlope = false;
        estimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), modulePositions, new Pose2d());
        new Thread(() -> {
            try {
                Thread.sleep(100);
                gyro.calibrate();
                resetGyro();
            } catch (Exception e) { }
        });
    }

    // on pit setup day, take robot to corner of field and reset (set 0, 0)
    public void resetGyro() {
        gyro.reset();
    }

    // updates the pose estimator, runs in periodic
    public void updateGyroOdometry() {
        if (!onSlope) { //if not on slope at start of tick
            if (getHeading().getY() > -minimumAngle && getHeading().getY() < minimumAngle) { //check if on slope
                startSlope(); //if on slope, switch into slope mode
            } 
            else { //if still not on slope
                normalCalc(); //calculate position normally
            }
        } else { //if on slope at start of tick
            if (getHeading().getY() < -minimumAngle || getHeading().getY() > minimumAngle) { //check if not on slope
                endSlope(); //if not on slope, switch out of slope mode
            }
            else { //if still on slope
                slopeCalc(); //calculate position based on slope
            }
        }

        lastX = getX();
    }

    public void normalCalc() {
        estimator.update(getRotation2d(), swerve.getModulePositions()); // regular estimator updating strategy
    }

    public void startSlope() {
        onSlope = true;

        slopeCalc();
    }

    // ;󠀿
    public void slopeCalc() {
        estimator.update(getRotation2d(), swerve.getModulePositions());
        double pdX = getX() - lastX;
        double adX = pdX * Math.cos(getHeading().getY());
        double aCurrentX = lastX + adX;
        Pose2d updatedPose = new Pose2d(new Translation2d(aCurrentX, getY()), getRotation2d());
        // hope and pray that this sets the current position to the updatedPose and doesn't just break things
        estimator.resetPosition(gyro.getRotation2d(), modulePositions, updatedPose);
    }

    public void endSlope() {
        onSlope = false;

        normalCalc();
    }

    // returns angular(?) values in rotation3d format
    public Rotation3d getHeading() {
        return new Rotation3d(
            toRadians(gyro.getRoll()), 
            toRadians(gyro.getPitch()), 
            toRadians(gyro.getYaw()));
    } 

    public Rotation2d getRotation2d() {
        // Negate the reading because the navX has CCW- and we need CCW+
        return Rotation2d.fromDegrees(Math.IEEEremainder(-gyro.getAngle(), 360));
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

    public double toRadians(float value) {
        double radians = value * Math.PI / 180;
        return radians;
    }
}