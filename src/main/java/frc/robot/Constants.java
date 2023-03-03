package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
    /** General info
     * Controller axis range: -1 to 1
     * Motor max: 5676 rot/min = 14.5 ft/s = 4.4196 m/s
     * Speed cap: 5000 rot/min
     * 
     * Gyro:
     *  - Forward = y+
     *  - Right = x+
     *  - Counterclockwise = z-
     */

    public static final Mode currentMode = Mode.REAL;
    public static final ModuleType powerDistributionType = ModuleType.kCTRE;
    public static final boolean fieldOriented = true;

    public static final double intakeSpeed = 2.0;
    public static final double shoulderSpeed = 1.0;
    public static final double elbowSpeed = 1.0;
    
    public static final class CAN {
        public static final int kIntakePort = 2;
        public static final int kShoulderPort = 0;
        public static final int kElbowPort = 1;

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 13;
        public static final int kBackRightDriveMotorPort = 14;

        public static final int kFrontLeftTurningMotorPort = 21;
        public static final int kFrontRightTurningMotorPort = 22;
        public static final int kBackLeftTurningMotorPort = 23;
        public static final int kBackRightTurningMotorPort = 24;
    }

    public static final class ModuleConstants {
        // Have to calculate using specs for the MK4i
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / (150 / 7.0);

        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final LoggedTunableNumber kPTurning = new LoggedTunableNumber("Turning kP", 0.5);
        public static final LoggedTunableNumber kITurning = new LoggedTunableNumber("Turning kI", 0);
        public static final LoggedTunableNumber kDTurning = new LoggedTunableNumber("Turning kD", 0);
    }

    public static final class DriveConstants {
        // Have to change depending on our robot design
        // Distance between right and left wheels:
        public static final double kTrackWidth = Units.inchesToMeters(26);
        // Distance between front and back wheels:
        public static final double kWheelBase = Units.inchesToMeters(26);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left (+/+)
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right (+/-)
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left (-/+)
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // Back right (-/-)

        // Edit depending on specs of MK4i
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = false;

        // Edit based on robot electrical system
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 4.305 + 2.600 - 2.149;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.172 - 2.892 + 2.485;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.268 - 1.794 + 1.865;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.610 + 2.111 - 2.119;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5 / 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final ControllerType controllerType = ControllerType.XBOX;
        
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = controllerType == ControllerType.XBOX ? 4 : 2;
        public static final int kDriverFieldOrientedButtonId = XboxController.Button.kStart.value;

        public static final double kDeadband = 0.15;
    }

    public static enum Mode {
        // Running on a real robot
        REAL,

        // Running a simulator
        SIM,

        // In tuning mode
        TUNING,

        // Replaying from a log file
        REPLAY
    }

    public static enum ControllerType {
        XBOX,
        LOGITECH
    }

    // fx = camera horizontal focal length (pixels)
    public static final Double fx = 384.922;
    // fy = camera vertical focal length (pixels)
    public static final Double fy = 384.922;
    // cx = camera horizontal focal center (pixels)
    public static final Double cx = 318.720;
    // cy = camera vertical focal center (pixels)
    public static final Double cy = 238.373;

    // size of AprilTag
    public static final Double tagSize = Units.inchesToMeters(8);
    // size of AprilTag image on Sid's computer with the pdf open in firefox at 100% zoom
    public static final Double testTagSize = Units.inchesToMeters(4 + (9/16));
}