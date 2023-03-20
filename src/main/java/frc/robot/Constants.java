package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.List;

import edu.wpi.first.apriltag.*;
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
    public static final ModuleType powerDistributionType = ModuleType.kRev;
    public static final boolean fieldOriented = true;

    public static final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Max intake speed", 0.2);
    public static final LoggedTunableNumber kMaxShoulderSpeed = new LoggedTunableNumber("Max shoulder speed", 0.1);
    public static final LoggedTunableNumber kMaxElbowSpeed = new LoggedTunableNumber("Max elbow speed", 0.1);

    public static final LoggedTunableNumber armP = new LoggedTunableNumber("arm p constant", 0.05);
     
    //arm absolute encoder ports
    public static final int kShoulderEncoderPort = 0;
    public static final int kElbowEncoderPort = 1;

    //arm absolute encoder offset
    public static final double kShoulderOffset = 0.753;
    public static final double kElbowOffset = 0.658;
 
    //stopping point for shoulder
    public static final double kShoulderBackLimit = -2.23;

    public static final double kStopShoulderForward = 5 * Math.PI / 6;
    public static final double kStopShoulderMid = Math.PI;
    public static final double kStopShoulderBackward = -5 * Math.PI / 6;

    public static final double kStopElbowForward = 5 * Math.PI / 6;
    public static final double kStopElbowMid = Math.PI;
    public static final double kStopElbowBackward = -5 * Math.PI / 6;

    // Arm to elebow gear ratio coefficient
    public static final double kArmsToElbow = -296 / 322;

    //arm preset points TODO: find these points
    public static final double[] kHighNodeSetpoints = new double[] { Math.PI / 2, Math.PI / 2};
    public static final double[] kMidNodeSetpoints = new double[] { 0, 0 };
    public static final double[] kHybridNodeSetpoints = new double[] { -Math.PI / 2, -Math.PI / 2 };
    public static final double[] kGroundIntakeSetpoints = new double[] { 0, 0 };
    public static final double[] kHighIntakeSetpoints = new double[] { 0, 0 };
    public static final double[] kStartingSetpoints = new double[] { 0, 0 };
    public static final double[] kTaxiSetpoints = new double[] { 0, 0 };

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
    // height of AprilTag at dropoff stations
    public static final Double tagHeight = Units.inchesToMeters(12 + (11 / 4));
    // size of AprilTag image on Sid's computer with the pdf open in firefox at 100% zoom
    public static final Double testTagSize = Units.inchesToMeters(4 + (9/16));

    public static final class CameraConstants {

        public static final Rotation3d[] cameraDirections = new Rotation3d[] { // camera mount directions
            new Rotation3d(0, 0, 0), 
            new Rotation3d(0, 0, Units.degreesToRadians(90)), 
            new Rotation3d(0, 0, Units.degreesToRadians(180))
        }; 
        // 1's are values we need to find
        // TODO: somebody other than chaerin with a working mechanical brain think about this, please?
        // please :()
        public static final Translation3d[] cameraDisplacements = new Translation3d[] { // camera mount position, from flat center
            new Translation3d(1, 1, 1), 
            new Translation3d(1, 1, 0), 
            new Translation3d(1, 1, 0) // height values are from the ground not from center of robot but others are from center of robot
        }; 
        public static final Transform3d[] cameraDistances = new Transform3d[] { 
            new Transform3d(cameraDisplacements[0], cameraDirections[0]), 
            new Transform3d(cameraDisplacements[1], cameraDirections[1]), 
            new Transform3d(cameraDisplacements[2], cameraDirections[2])
        };
        
        // bad test values. update when actually mounted.
    }

    public static final class LoadingZone {
    // Region dimensions
    public static final double width = Units.inchesToMeters(99.0);
    public static final double innerX = FieldConstants.fieldLength;
    public static final double midX = FieldConstants.fieldLength - Units.inchesToMeters(132.25);
    public static final double outerX = FieldConstants.fieldLength - Units.inchesToMeters(264.25);
    public static final double leftY = FieldConstants.fieldWidth;
    public static final double midY = leftY - Units.inchesToMeters(50.5);
    public static final double rightY = leftY - width;
    public static final Translation2d[] regionCorners =
        new Translation2d[] {
          new Translation2d(
              midX, rightY), // Start at lower left next to border with opponent community
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
    public static final double singleSubstationLeftX =
        FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
    public static final double singleSubstationCenterX =
        singleSubstationLeftX + (singleSubstationWidth / 2.0);
    public static final double singleSubstationRightX =
        singleSubstationLeftX + singleSubstationWidth;
    public static final Translation2d singleSubstationTranslation =
        new Translation2d(singleSubstationCenterX, leftY);

    public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
    public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
    public static final double singleSubstationCenterZ =
        singleSubstationLowZ + (singleSubstationHeight / 2.0);
    public static final double singleSubstationHighZ =
        singleSubstationLowZ + singleSubstationHeight;
  }

      public static final class Grids {
        // X layout
        public static final double outerX = Units.inchesToMeters(54.25);
        public static final double lowX =
            outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
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

        static {
        for (int i = 0; i < nodeRowCount; i++) {
            boolean isCube = i == 1 || i == 4 || i == 7;
            lowTranslations[i] = new Translation2d(lowX, nodeY[i]);
            low3dTranslations[i] = new Translation3d(lowX, nodeY[i], 0.0);
            midTranslations[i] = new Translation2d(midX, nodeY[i]);
            mid3dTranslations[i] = new Translation3d(midX, nodeY[i], isCube ? midCubeZ : midConeZ);
            highTranslations[i] = new Translation2d(highX, nodeY[i]);
            high3dTranslations[i] = new Translation3d(highX, nodeY[i], isCube ? highCubeZ : highConeZ);
        }
        }
        // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
        public static final double complexLowXCones =
            outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
        public static final double complexLowXCubes = lowX; // Centered X under cube nodes
        public static final double complexLowOuterYOffset =
            nodeY[0] - (Units.inchesToMeters(3.0) + (Units.inchesToMeters(25.75) / 2.0));

        public static final Translation2d[] complexLowTranslations =
            new Translation2d[] {
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

        public static final Translation3d[] complexLow3dTranslations =
            new Translation3d[] {
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
    
    public static final class FieldConstants {
        public static final boolean isWPIField = false; // Red alliance
        public static final double fieldLength = Units.inchesToMeters(651.25);
        public static final double fieldWidth =
            Units.inchesToMeters(315.5) + (isWPIField ? Units.inchesToMeters(3.0) : 0.0);
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
}