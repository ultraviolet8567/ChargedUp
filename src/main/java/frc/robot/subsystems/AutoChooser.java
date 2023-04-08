package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoLock;
import frc.robot.commands.auto.AutoPlace;
import frc.robot.odometry.GyroOdometry;
import frc.robot.Robot;

public class AutoChooser extends SubsystemBase {
    private static final ShuffleboardTab main = Shuffleboard.getTab("Main");

    private final Swerve swerve;
    private final Arms arms;
    private final Intake intake;
    private final GyroOdometry gyro;

    private final SendableChooser<Boolean> placeGamePiece;
    private final SendableChooser<String> chargeStation;

    public AutoChooser(Swerve swerve, Arms arms, Intake intake, GyroOdometry gyro) {
        this.swerve = swerve;
        this.arms = arms;
        this.intake = intake;
        this.gyro = gyro;

        chargeStation = new SendableChooser<>();
        chargeStation.setDefaultOption("Drive out", "Drive out");
        chargeStation.addOption("Balance", "Balance");
        chargeStation.addOption("Don't move", "Don't move");
        main.add("Balance on charge station", chargeStation).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(4, 2);

        placeGamePiece = new SendableChooser<>();
        placeGamePiece.setDefaultOption("No", false);
        placeGamePiece.addOption("Yes", true);
        main.add("Place game piece", placeGamePiece).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(4, 3);
    }

    public void periodic() {
        Logger.getInstance().recordOutput("Auto/Path", getPathName());
    }

    public String getPathName() {
        String pathName = "";
        
        if (placeGamePiece.getSelected()) {
            switch (Robot.getGamePiece()) {
                case CONE:
                    pathName += "ConePlace";
                    break;
                case CUBE:
                default:
                    pathName += "CubePlace";
                    break;
            }
        }

        if (chargeStation.getSelected().equals("Balance")) {
            pathName += "Balance";
        }
        else if (chargeStation.getSelected().equals("Drive out")) {
            pathName += "DriveOut";
        }

        if (pathName.equals(""))
            pathName = "DriveOut";

        if (DriverStation.getAlliance() == Alliance.Red)
            pathName += "Red";
        else
            pathName += "Blue";

        return pathName;
    }

    public PathPlannerTrajectory getTrajectory() {
        if (getPathName().contains("Balance")) {
            return PathPlanner.loadPath(getPathName(), new PathConstraints(1.5 * AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        }
        else {
            return PathPlanner.loadPath(getPathName(), new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
        }
    }

    public PPSwerveControllerCommand getControllerCommand() {
        PathPlannerTrajectory trajectory = getTrajectory();

        return new PPSwerveControllerCommand(
            trajectory, 
            gyro::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(AutoConstants.kPXController, 0, 0), // X controller. Leaving it at 0 will only use feedforwards.
            new PIDController(AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.kPThetaController, 0, 0), // Rotation controller. Leaving it at 0 will only use feedforwards.
            swerve::setModuleStates, // Module states consumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // Requires this drive subsystem
        );
    }

    public Command getAutoCommand() {
        if (chargeStation.getSelected().equals("Balance")) {
            if (placeGamePiece.getSelected()) {
                // Place on high node and balance
                Logger.getInstance().recordOutput("Auto/Routine", "Place on high node and balance");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialPose())),
                    new AutoPlace(arms, intake),
                    getControllerCommand(),
                    new AutoBalance(swerve, gyro),
                    new AutoLock(swerve));
            }
            else {
                // Balance on charge station
                Logger.getInstance().recordOutput("Auto/Routine", "Balance on charge station");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialPose())),
                    getControllerCommand(),
                    new AutoBalance(swerve, gyro),
                    new AutoLock(swerve));
            }
        }
        else if (chargeStation.getSelected().equals("DriveOut")) {
            if (placeGamePiece.getSelected()) {
                // Place on high node and drive out
                Logger.getInstance().recordOutput("Auto/Routine", "Place on high node and drive out");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialPose())),
                    new AutoPlace(arms, intake),
                    getControllerCommand(),
                    new AutoLock(swerve));
            }
            else {
                // Drive out
                Logger.getInstance().recordOutput("Auto/Routine", "Drive out");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialPose())),
                    getControllerCommand(),
                    new AutoLock(swerve));
            }
        }
        else {
            if (placeGamePiece.getSelected()) {
                // Place on high node and do not move
                Logger.getInstance().recordOutput("Auto/Routine", "Place on high node and do nothing");
                return new SequentialCommandGroup(
                    new AutoPlace(arms, intake),
                    new AutoLock(swerve));
            }
            else {
                // Do nothing
                Logger.getInstance().recordOutput("Auto/Routine", "Do nothing");
                return null;
            }
        }
    }
}
