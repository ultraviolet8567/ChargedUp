package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

    private double desiredCardinalAngle;
    private boolean cardinalDirectionEnabled;
    private PIDController cardinalPidController;

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

        cardinalPidController = new PIDController(0.1, 0, 0);
        cardinalPidController.setTolerance(0.1);
        cardinalPidController.enableContinuousInput(-Math.PI, Math.PI);
        cardinalDirectionEnabled = false;
    }

    public void periodic() {
        // FL angle, FL speed, FR angle, FR speed, BL angle, BL speed, BR angle, BR speed
        Logger.getInstance().recordOutput("Measured/SwerveModuleStates", new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState() });

        // FL absolute encoder angle, FR absolute encoder angle, BL absolute encoder angle, BR absolute encoder angle
        Logger.getInstance().recordOutput("AbsoluteEncoders/Swerve", new double[] { frontLeft.getAbsoluteEncoderAngle(), frontRight.getAbsoluteEncoderAngle(), backLeft.getAbsoluteEncoderAngle(), backRight.getAbsoluteEncoderAngle() });

        Logger.getInstance().recordOutput("AtCardinalDirection", cardinalPidController.atSetpoint());
        Logger.getInstance().recordOutput("OnCardinal", cardinalDirectionEnabled);
        Logger.getInstance().recordOutput("DesiredCardinalAngle", desiredCardinalAngle);
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

    public boolean cardinalDirectionEnabled() {
        return cardinalDirectionEnabled;
    }

    public void setCardinalDirection(double desiredAngle) {
        desiredCardinalAngle = desiredAngle;
        cardinalDirectionEnabled = true;
    }

    public void disableCardinalDirection() {
        cardinalDirectionEnabled = false;
    }

    public double getTurningSpeed(double angle) {
        return cardinalPidController.calculate(angle, desiredCardinalAngle);
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