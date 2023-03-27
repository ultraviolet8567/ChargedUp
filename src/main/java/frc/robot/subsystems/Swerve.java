package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DriveConstants;

public class Swerve extends SubsystemBase {
    private final SwerveModule frontLeft, frontRight, backLeft, backRight;

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
    }

    public void periodic() {
        // FL angle, FL speed, FR angle, FR speed, BL angle, BL speed, BR angle, BR speed
        Logger.getInstance().recordOutput("Measured/SwerveModuleStates", new SwerveModuleState[] { frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState() });

        // FL absolute encoder angle, FR absolute encoder angle, BL absolute encoder angle, BR absolute encoder angle
        Logger.getInstance().recordOutput("AbsoluteEncoders/Swerve", new double[] { frontLeft.getAbsoluteEncoderAngle(), frontRight.getAbsoluteEncoderAngle(), backLeft.getAbsoluteEncoderAngle(), backRight.getAbsoluteEncoderAngle() });
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] output = {frontLeft.getModulePosition(), frontRight.getModulePosition(), backLeft.getModulePosition(), backRight.getModulePosition()};
        return output;
    }
    
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        Logger.getInstance().recordOutput("Setpoints/SwerveModuleStates", desiredStates);
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