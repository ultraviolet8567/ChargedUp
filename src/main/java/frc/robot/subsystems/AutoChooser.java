package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
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

public class AutoChooser extends SubsystemBase {
    private static final ShuffleboardTab main = Shuffleboard.getTab("Main");

    private final Swerve swerve;
    private final Arms arms;
    private final Intake intake;
    private final GyroOdometry gyro;

    private final SendableChooser<Boolean> chargeStation;
    private final SendableChooser<String> placeGamePiece;

    public AutoChooser(Swerve swerve, Arms arms, Intake intake, GyroOdometry gyro) {
        this.swerve = swerve;
        this.arms = arms;
        this.intake = intake;
        this.gyro = gyro;

        chargeStation = new SendableChooser<>();
        chargeStation.setDefaultOption("No", false);
        chargeStation.addOption("Yes", true);
        main.add("Balance on charge station", chargeStation).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(4, 2);

        placeGamePiece = new SendableChooser<>();
        placeGamePiece.setDefaultOption("None", "None");
        placeGamePiece.addOption("Cone", "Cone");
        placeGamePiece.addOption("Cube", "Cube");
        main.add("Place game piece", placeGamePiece).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(4, 3);
    }

    public String getPathName() {
        String pathName = "";
        
        switch (placeGamePiece.getSelected()) {
            case "Cone":
                pathName += "ConePlace";
                break;
            case "Cube":
                pathName += "CubePlace";
                break;
            default:
                break;
        }

        if (chargeStation.getSelected()) {
            pathName += "Balance";
        }
        else {
            pathName += "DriveOut";
        }
        
        return pathName;
    }

    public PathPlannerTrajectory getTrajectory() {
        return PathPlanner.loadPath(getPathName(), new PathConstraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared));
    }

    public PPSwerveControllerCommand getControllerCommand() {
        PathPlannerTrajectory trajectory = getTrajectory();

        return new PPSwerveControllerCommand(
            trajectory, 
            gyro::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            swerve::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // Requires this drive subsystem
        );
    }

    public Command getAutoCommand() {
        if (chargeStation.getSelected()) {
            if (placeGamePiece.getSelected().equals("None")) {
                // Balance on charge station
                Logger.getInstance().recordOutput("Auto/Routine", "Balance on charge station");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialHolonomicPose())),
                    getControllerCommand(),
                    new AutoBalance(swerve, gyro),
                    new AutoLock(swerve),
                    new InstantCommand(() -> swerve.stopModules()));
            }
            else {
                // Place on high node and balance
                Logger.getInstance().recordOutput("Auto/Routine", "Place on high node and balance");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialHolonomicPose())),
                    new AutoPlace(arms, intake),
                    getControllerCommand(),
                    new AutoBalance(swerve, gyro),
                    new AutoLock(swerve),
                    new InstantCommand(() -> swerve.stopModules()));
            }
        }
        else {
            if (placeGamePiece.getSelected().equals("None")) {
                // Drive out
                Logger.getInstance().recordOutput("Auto/Routine", "Drive out");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialHolonomicPose())),
                    getControllerCommand(),
                    new AutoLock(swerve),
                    new InstantCommand(() -> swerve.stopModules()));
            }
            else {
                // Place on high node and drive out
                Logger.getInstance().recordOutput("Auto/Routine", "Place on high node and drive out");
                return new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(getTrajectory().getInitialHolonomicPose())),
                    new AutoPlace(arms, intake),
                    getControllerCommand(),
                    new AutoBalance(swerve, gyro),
                    new AutoLock(swerve),
                    new InstantCommand(() -> swerve.stopModules()));
            }
        }
    }
}
