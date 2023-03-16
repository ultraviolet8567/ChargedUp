package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.ControllerIO.ControllerType;

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

    public static final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Max intake speed", 0.2);
    public static final LoggedTunableNumber shoulderSpeed = new LoggedTunableNumber("Max shoulder speed", 0.1);
    public static final LoggedTunableNumber elbowSpeed = new LoggedTunableNumber("Max elbow speed", 0.2);
    
    //arm absolute encoder ports
    public static final int kShoulderEncoderPort = 0;
    public static final int kElbowEncoderPort = 1;

    //arm absolute encoder offset
    public static final double kShoulderOffset = 0.668;
    public static final double kElbowOffset = 0.183;

    //stopping point for shoulder
    public static final int kStopShoulderForward = 220;
    public static final int kStopShoulderBackward = 300;
    public static final int kStopShoulderMid = 260;

    public static final int kStopElbowBackward = 135;
    public static final int kStopElbowForward = 225;
    public static final int kStopElbowMid = 180;

    //arm preset points TODO: find these points
    public static final int[] kHighNodeSetpoints = new int[] { 50, 0 };
    public static final int[] kMidNodeSetpoints = new int[] { 50, 0 };
    public static final int[] kHybridNodeSetpoints = new int[] { 50, 0 };
    public static final int[] kGroundIntakeSetpoints = new int[] { 50, 0 };
    public static final int[] kHighIntakeSetpoints = new int[] { 50, 0 };
    public static final int[] kStartingSetpoints = new int[] { 50, 0 };
    public static final int[] kTaxiSetpoints = new int[] { 50, 0 };

    public static final class CAN {
        public static final int kIntakePort = 3;
        public static final int kElbowPort = 2;
        public static final int kShoulderPort = 4;

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 14;
        public static final int kBackRightDriveMotorPort = 13;

        public static final int kFrontLeftTurningMotorPort = 21;
        public static final int kFrontRightTurningMotorPort = 22;
        public static final int kBackLeftTurningMotorPort = 24;
        public static final int kBackRightTurningMotorPort = 23;
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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -7.378 + 1.319 - 1.651 - 2.979 - 0.141 + 0.253 - 1.694;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -1.205 - 1.161 - 0.724 - 2.476 + 2.4375 + 0.032;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -1.114 - 1.206 + 1.262 - 3.772 - 0.634 - 1.889 + 0.0932 - 0.037;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -1.258 - 1.195 + 0.542 - 2.192 + 3.053 - 0.913 - 0.010 - 0.058;
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5 / 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 2;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2 * 0.4;
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
        public static final ControllerType controllerType = ControllerType.JOYSTICK;
        
        public static final int kDriverControllerPort = 0;
        public static final int kToggleControllerPort = 1;

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