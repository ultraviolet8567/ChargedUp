package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.odometry.GyroOdometry;

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private int desiredAngle;

    public boolean atCardinalDirection;

    private PIDController cardnalPidController;

    public Swerve() {
        frontLeft = new SwerveModule(
            CAN.kFrontLeftDriveMotorPort,
            CAN.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

        frontRight = new SwerveModule(
            CAN.kFrontRightDriveMotorPort,
            CAN.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
        
        backLeft = new SwerveModule(
            CAN.kBackLeftDriveMotorPort,
            CAN.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
        
        backRight = new SwerveModule(
            CAN.kBackRightDriveMotorPort,
            CAN.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

        cardnalPidController = new PIDController(0, 0, 0);
        atCardinalDirection = true;
    }

    public void periodic() {
        // FL angle, FL speed, FR angle, FR speed, BL angle, BL speed, BR angle, BR speed
        Logger.getInstance().recordOutput("Measured/SwerveModuleStates", new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState() });

        // FL absolute encoder angle, FR absolute encoder angle, BL absolute encoder angle, BR absolute encoder angle
        Logger.getInstance().recordOutput("AbsoluteEncoders/Swerve", new double[] { frontLeft.getAbsoluteEncoderAngle(), frontRight.getAbsoluteEncoderAngle(), backLeft.getAbsoluteEncoderAngle(), backRight.getAbsoluteEncoderAngle() });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(), backRight.getModulePosition() };
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        Logger.getInstance().recordOutput("Setpoints/SwerveModuleStates", desiredStates);
    }

    // Sets the wheels to 45 degree angles so it doesn't move
    public void lockWheels() {
        SwerveModuleState[] locked = new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(2.386)),
            new SwerveModuleState(0, new Rotation2d(0.755)), 
            new SwerveModuleState(0, new Rotation2d(-2.386)), 
            new SwerveModuleState(0, new Rotation2d(-0.755))
        };

        setModuleStates(locked);
    }

    public void setCardinalDirection(int desiredAngle) {
        this.desiredAngle = desiredAngle;
        atCardinalDirection = false;
    }

    public double getTurningSpeed() {
        if(cardnalPidController.atSetpoint()){
            atCardinalDirection = true;
            return 0.0;
        }

        return cardnalPidController.calculate(GyroOdometry.getHeading(), desiredAngle);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
}