package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.util.LoggedTunableNumber;

public final class Constants {
    /** General info
     * Controller axis range: -1 to 1
     * Motor max: 5676 rot/min = 14.5 ft/s = 4.5 m/s
     * Speed cap: 5000 rot/min
     * 
     * Gyro:
     *  - Forward = y+
     *  - Left = x-
     *  - Counterclockwise = z-
     * 
     * Odometry
     *  - Forward = x+
     *  - Left = y+
     *  - Counterclockwise = z+ 
     */

    public static final Mode currentMode = Mode.REAL;
    public static final ModuleType powerDistributionType = ModuleType.kRev;
    public static final boolean fieldOriented = true;
    public static final String logpath = "/media/sda1/";
    
    // Offset subtracted from the gyro reading so that 0 is forward when on the Red Alliance
    public static final double kGyroOffsetRed = 0;
    // Make sure to take the opposite as 0 if on Blue Alliance
    public static final double kGyroOffset = DriverStation.getAlliance() == Alliance.Red ? kGyroOffsetRed : kGyroOffsetRed + Math.PI;

    public static final class ArmConstants {
        public static final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Max intake speed", 0.5);

        // Rotation boundaries for arm joints point
        public static final double kShoulderFrontLimit = 2.693;
        public static final double kShoulderFrontMechanicalStop = 2.968;
        public static final double kShoulderBackLimit = -2.248;
        public static final double kShoulderBackMechanicalStop = -2.487;
        public static final double kElbowFrontLimit = 2.438;
        public static final double kElbowFrontMechanicalStop = 2.647; 
        public static final double kElbowBackLimit = -2.581;
        public static final double kElbowBackMechanicalStop = -2.792;
        
        public static final double kMaxShoulderSpeedPercentage = 0.8;
        public static final double kMaxElbowSpeedPercentage = 0.8;

        // PID control constants
        public static final LoggedTunableNumber kMaxShoulderSpeed = new LoggedTunableNumber("Max shoulder speed", 3.5);
        public static final LoggedTunableNumber kMaxShoulderAcceleration = new LoggedTunableNumber("Max shoulder acceleration", 1);
        public static final LoggedTunableNumber kPShoulder = new LoggedTunableNumber("Shoulder kP", 1);
        public static final LoggedTunableNumber kIShoulder = new LoggedTunableNumber("Shoulder kI", 0);
        public static final LoggedTunableNumber kDShoulder = new LoggedTunableNumber("Shoulder kD", 0);
        public static final double kShoulderPidTolerance = (kShoulderFrontLimit - kShoulderBackLimit) / 50.0;
        
        public static final LoggedTunableNumber kMaxElbowSpeed = new LoggedTunableNumber("Max elbow speed", 0.8);
        public static final LoggedTunableNumber kMaxElbowAcceleration = new LoggedTunableNumber("Max elbow acceleration", 1);
        public static final LoggedTunableNumber kPElbow = new LoggedTunableNumber("Elbow kP", 1);
        public static final LoggedTunableNumber kIElbow = new LoggedTunableNumber("Elbow kI", 0);
        public static final LoggedTunableNumber kDElbow = new LoggedTunableNumber("Elbow kD", 0);
        public static final double kElbowPidTolerance = (kElbowFrontLimit - kElbowBackLimit) / 150.0;

        // Arm absolute encoders
        public static final int kShoulderEncoderPort = 0;
        public static final int kElbowEncoderPort = 1;
    
        // Encoder offsets
        public static final double kShoulderEncoderOffset = 0;
        public static final double kElbowEncoderOffset = -1.705;

        // Arm to elbow gear ratio coefficient
        public static final double kArmsToElbow = -152.0 / 322.0;

        // TODO: find these points (Jackson and Aadi)
        public static final double[] kHighNodeConeSetpoints = new double[] { 0.850, 0.454 };
        public static final double[] kHighNodeCubeSetpoints = new double[] { 0.715, 0.608 };
        public static final double[] kMidNodeConeSetpoints = new double[] { -0.233, 2.024 };
        public static final double[] kMidNodeCubeSetpoints = new double[] { -0.380, 1.949 };
        public static final double[] kHybridNodeConeSetpoints = new double[] { 2.796, -1.183 };
        public static final double[] kHybridNodeCubeSetpoints = new double[] { 2.796, -1.183 };
        public static final double[] kGroundIntakeConeSetpoints = new double[] { 2.773, -1.114 };
        public static final double[] kGroundIntakeCubeSetpoints = new double[] { 2.975, -1.032 };
        public static final double[] kSubstationIntakeConeSetpoints = new double[] { -0.601, 2.041 };
        public static final double[] kSubstationIntakeCubeSetpoints = new double[] { -0.474, 1.998 };
        public static final double[] kStartSetpoints = new double[] { -2.278, 2.427 };
        public static final double[] kTaxiSetpoints = new double[] { -2.049, 2.35 };
    }

    public static final class OIConstants {
        public static final ControllerType controllerTypeDrive = ControllerType.XBOX;
        public static final ControllerType controllerTypeArms = ControllerType.XBOX;
        
        public static final int kDriveControllerPort = 0;
        public static final int kArmControllerPort = 1;

        public static final double kDeadband = 0.1;
    }

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

        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {
        // Have to change depending on our robot design
        // Distance between right and left wheels:
        public static final double kTrackWidth = Units.inchesToMeters(34); // 0.8636 m
        // Distance between front and back wheels:
        public static final double kWheelBase = Units.inchesToMeters(32); // 0.8128 m
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

        // Temporarily negate to make the video work (hot fix, don't forget to reverse)
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

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 16.972;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 3.2845;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 7.2058;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.883;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * Math.PI;

        public static double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond * 0.9;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond * 0.4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kPXController = 0.25;
        public static final double kPYController = 0.25;
        public static final double kPThetaController = 0.25;
    }

    public static final class Camera {
        // Update these constants when cameras are mounted

        public static final Rotation3d[] cameraDirections = new Rotation3d[] { // camera mount directions
            new Rotation3d(0, 0, Units.degreesToRadians(-135)), // front camera
            new Rotation3d(0, 0, Units.degreesToRadians(-46)), // right camera
            new Rotation3d(0, 0, 0) // left camera
        }; 
        // 1's are values we need to find
        // TODO: somebody other than chaerin with a working mechanical brain think about this, please?
        // please :(
        public static final Translation3d[] cameraDisplacements = new Translation3d[] { // camera mount position, from flat center
            new Translation3d(Units.inchesToMeters(6.5), Units.inchesToMeters(2.6875), Units.inchesToMeters(13.5)), // front camera
            new Translation3d(Units.inchesToMeters(9.75), Units.inchesToMeters(9.25), Units.inchesToMeters(5.35)), // right camera
            new Translation3d(1, 1, 0) // left camera
        };
        public static final Transform3d[] cameraDistances = new Transform3d[] { 
            new Transform3d(cameraDisplacements[0], cameraDirections[0]), 
            new Transform3d(cameraDisplacements[1], cameraDirections[1]), 
            new Transform3d(cameraDisplacements[2], cameraDirections[2])
        };
    }

    public static final class FieldConstants {

        public static final class LoadingZone {
            // Region dimensions
            public static final double width = Units.inchesToMeters(99.0);
            public static final double innerX = FieldConstants.fieldLength;
            public static final double midX = FieldConstants.fieldLength - Units.inchesToMeters(132.25);
            public static final double outerX = FieldConstants.fieldLength - Units.inchesToMeters(264.25);
            public static final double leftY = FieldConstants.fieldWidth;
            public static final double midY = leftY - Units.inchesToMeters(50.5);
            public static final double rightY = leftY - width;
            public static final Translation2d[] regionCorners = new Translation2d[] {
                new Translation2d(midX, rightY), // Start at lower left next to border with opponent community
                new Translation2d(midX, midY),
                new Translation2d(outerX, midY),
                new Translation2d(outerX, leftY),
                new Translation2d(innerX, leftY),
                new Translation2d(innerX, rightY),
            };
        
            // Double substation dimensions
            public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
            public static final double doubleSubstationX = innerX - doubleSubstationLength;
            public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
            public static final double doubleSubstationCenterY = FieldConstants.fieldWidth - Units.inchesToMeters(49.76);
        
            // Single substation dimensions
            public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
            public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
            public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
            public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
            public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX, leftY);
        
            public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
            public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
            public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
            public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
            }
    
        public static final class Grids {
            // X layout
            public static final double outerX = Units.inchesToMeters(54.25);
            public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
            public static final double midX = outerX - Units.inchesToMeters(22.75);
            public static final double highX = outerX - Units.inchesToMeters(39.75);
    
            // Y layout
            public static final int nodeRowCount = 9;
            public static final double[] nodeY =
                FieldConstants.isWPIField
                    ? new double[] {
                    Units.inchesToMeters(20.19 + 22.0 * 0),
                    Units.inchesToMeters(20.19 + 22.0 * 1),
                    Units.inchesToMeters(20.19 + 22.0 * 2),
                    Units.inchesToMeters(20.19 + 22.0 * 3),
                    Units.inchesToMeters(20.19 + 22.0 * 4),
                    Units.inchesToMeters(20.19 + 22.0 * 5),
                    Units.inchesToMeters(20.19 + 22.0 * 6),
                    Units.inchesToMeters(20.19 + 22.0 * 7),
                    Units.inchesToMeters(20.19 + 22.0 * 8 + 2.5)
                    }
                    : new double[] {
                    Units.inchesToMeters(20.19 + 22.0 * 0),
                    Units.inchesToMeters(20.19 + 22.0 * 1),
                    Units.inchesToMeters(20.19 + 22.0 * 2),
                    Units.inchesToMeters(20.19 + 22.0 * 3),
                    Units.inchesToMeters(20.19 + 22.0 * 4),
                    Units.inchesToMeters(20.19 + 22.0 * 5),
                    Units.inchesToMeters(20.19 + 22.0 * 6),
                    Units.inchesToMeters(20.19 + 22.0 * 7),
                    Units.inchesToMeters(20.19 + 22.0 * 8)
                    };
    
            // Z layout
            public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
            public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
            public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
            public static final double highConeZ = Units.inchesToMeters(46.0);
            public static final double midConeZ = Units.inchesToMeters(34.0);
    
            // Translations (all nodes in the same column/row have the same X/Y coordinate)
            public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] low3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
            public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
            public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];
    
            static { for (int i = 0; i < nodeRowCount; i++) {
                boolean isCube = i == 1 || i == 4 || i == 7;
                lowTranslations[i] = new Translation2d(lowX, nodeY[i]);
                low3dTranslations[i] = new Translation3d(lowX, nodeY[i], 0.0);
                midTranslations[i] = new Translation2d(midX, nodeY[i]);
                mid3dTranslations[i] = new Translation3d(midX, nodeY[i], isCube ? midCubeZ : midConeZ);
                highTranslations[i] = new Translation2d(highX, nodeY[i]);
                high3dTranslations[i] = new Translation3d(highX, nodeY[i], isCube ? highCubeZ : highConeZ);
            }}
    
            // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
            public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
            public static final double complexLowXCubes = lowX; // Centered X under cube nodes
            public static final double complexLowOuterYOffset = nodeY[0] - (Units.inchesToMeters(3.0) + (Units.inchesToMeters(25.75) / 2.0));
    
            public static final Translation2d[] complexLowTranslations = new Translation2d[] {
                new Translation2d(complexLowXCones, nodeY[0] - complexLowOuterYOffset),
                new Translation2d(complexLowXCubes, nodeY[1]),
                new Translation2d(complexLowXCones, nodeY[2]),
                new Translation2d(complexLowXCones, nodeY[3]),
                new Translation2d(complexLowXCubes, nodeY[4]),
                new Translation2d(complexLowXCones, nodeY[5]),
                new Translation2d(complexLowXCones, nodeY[6]),
                new Translation2d(complexLowXCubes, nodeY[7]),
                new Translation2d(complexLowXCones, nodeY[8] + complexLowOuterYOffset),
            };
    
            public static final Translation3d[] complexLow3dTranslations = new Translation3d[] {
                new Translation3d(complexLowXCones, nodeY[0] - complexLowOuterYOffset, 0.0),
                new Translation3d(complexLowXCubes, nodeY[1], 0.0),
                new Translation3d(complexLowXCones, nodeY[2], 0.0),
                new Translation3d(complexLowXCones, nodeY[3], 0.0),
                new Translation3d(complexLowXCubes, nodeY[4], 0.0),
                new Translation3d(complexLowXCones, nodeY[5], 0.0),
                new Translation3d(complexLowXCones, nodeY[6], 0.0),
                new Translation3d(complexLowXCubes, nodeY[7], 0.0),
                new Translation3d(complexLowXCones, nodeY[8] + complexLowOuterYOffset, 0.0),
            };
        }

        public static final class AprilTagDimensions {
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
            // height of AprilTag at dropoff stations
            public static final Double tagHeight = Units.inchesToMeters(12 + (11 / 4));
            // size of AprilTag image on Sid's computer with the pdf open in firefox at 100% zoom
            public static final Double testTagSize = Units.inchesToMeters(4 + (9/16));
        }
        
        public static final boolean isWPIField = false; // Red alliance
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth = Units.inchesToMeters(315.5) + (isWPIField ? Units.inchesToMeters(3.0) : 0.0);
        public static final AprilTagFieldLayout aprilTags =
            isWPIField
            ? new AprilTagFieldLayout(
                List.of(
                    new AprilTag(
                        1,
                        new Pose3d(
                            Units.inchesToMeters(610.125),
                            Units.inchesToMeters(43.5),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        2,
                        new Pose3d(
                            Units.inchesToMeters(610.375),
                            Units.inchesToMeters(109.5),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        3,
                        new Pose3d(
                            Units.inchesToMeters(610.0),
                            Units.inchesToMeters(176.0),
                            Units.inchesToMeters(19.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        4,
                        new Pose3d(
                            Units.inchesToMeters(635.375),
                            Units.inchesToMeters(272.0),
                            Units.inchesToMeters(27.25),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        5,
                        new Pose3d(
                            Units.inchesToMeters(14.25),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d())),
                    new AprilTag(
                        6,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        7,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        8,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d()))),
                fieldLength,
                fieldWidth)
            : new AprilTagFieldLayout(
                List.of(
                    new AprilTag(
                        1,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        2,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        3,
                        new Pose3d(
                            Units.inchesToMeters(610.77),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        4,
                        new Pose3d(
                            Units.inchesToMeters(636.96),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d(0.0, 0.0, Math.PI))),
                    new AprilTag(
                        5,
                        new Pose3d(
                            Units.inchesToMeters(14.25),
                            LoadingZone.doubleSubstationCenterY,
                            Units.inchesToMeters(27.38),
                            new Rotation3d())),
                    new AprilTag(
                        6,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[7],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        7,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[4],
                            Units.inchesToMeters(18.22),
                            new Rotation3d())),
                    new AprilTag(
                        8,
                        new Pose3d(
                            Units.inchesToMeters(40.45),
                            Grids.nodeY[1],
                            Units.inchesToMeters(18.22),
                            new Rotation3d()))),
                fieldLength,
                fieldWidth);
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
        LOGITECH,
        JOYSTICK
    }
    
    public static enum GamePiece {
        CONE,
        CUBE
    }

    public static enum Preset {
        HIGH_NODE,
        MID_NODE,
        HYBRID_NODE,
        GROUND_INTAKE,
        SUBSTATION_INTAKE,
        START,
        TAXI,
        IDLE,
        MANUAL_OVERRIDE
    }
}