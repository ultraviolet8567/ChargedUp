package frc.robot.subsystems;

import static java.util.Map.entry;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoBalance;
import frc.robot.commands.auto.AutoLock;
import frc.robot.commands.auto.AutoPlace;
import frc.robot.odometry.GyroOdometry;

public class AutoChooser extends SubsystemBase {
    private static final ShuffleboardTab main = Shuffleboard.getTab("Main");

    private final Swerve swerve;
    private final GyroOdometry gyro;

    private final AutoBalance autoBalance;
    private final AutoPlace autoPlace;
    private final AutoLock autoLock;

    private final Map<String, Command> routines;

    private final SendableChooser<Boolean> placeGamePiece;
    private final SendableChooser<String> chargeStation;

    public AutoChooser(Swerve swerve, Arms arms, Intake intake, GyroOdometry gyro) {
        this.swerve = swerve;
        this.gyro = gyro;

        autoBalance = new AutoBalance(swerve, gyro);
        autoPlace = new AutoPlace(arms, intake);
        autoLock = new AutoLock(swerve);

        // Post the selectors to the ShuffleBoard
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

        // Instantiate all of the routines
        String alliance = DriverStation.getAlliance() == Alliance.Red ? "Red" : "Blue";
        routines = Map.ofEntries(
            entry("Cone place on high node and balance",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("ConePlaceBalance" + alliance).getInitialPose())),
                    autoPlace,
                    getControllerCommand("ConePlaceBalance" + alliance),
                    autoBalance,
                    autoLock)),
            entry("Cube place on high node and balance",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("CubePlaceBalance" + alliance).getInitialPose())),
                    autoPlace,
                    getControllerCommand("CubePlaceBalance" + alliance),
                    autoBalance,
                    autoLock)),
            entry("Balance on charge station",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("Balance" + alliance).getInitialPose())),
                    getControllerCommand("Balance" + alliance),
                    autoBalance,
                    autoLock)),
            entry("Cone place on high node and drive out",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("ConePlaceDriveOut" + alliance).getInitialPose())),
                    autoPlace,
                    getControllerCommand("ConePlaceDriveOut" + alliance),
                    autoLock)),
            entry("Cube place on high node and drive out",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("CubePlaceDriveOut" + alliance).getInitialPose())),
                    autoPlace,
                    getControllerCommand("CubePlaceDriveOut" + alliance),
                    autoLock)),
            entry("Drive out",
                new SequentialCommandGroup(
                    new InstantCommand(() -> gyro.resetOdometry(Constants.trajectories.get("DriveOut" + alliance).getInitialPose())),
                    getControllerCommand("DriveOut" + alliance),
                    autoLock)),
            entry("Place on high node and do not move",
                new SequentialCommandGroup(
                    autoPlace,
                    autoLock)),
            entry("Do nothing",
                null));
    }

    public PPSwerveControllerCommand getControllerCommand(String pathName) {
        return new PPSwerveControllerCommand(
            Constants.trajectories.get(pathName), 
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
        String gamePiece = Robot.toString(Robot.getGamePiece());

        if (chargeStation.getSelected().equals("Balance")) {
            if (placeGamePiece.getSelected()) {
                // Place on high node and balance
                Logger.getInstance().recordOutput("Auto/Routine", gamePiece + " place on high node and balance");
                return routines.get(gamePiece + " place on high node and balance");
            }
            else {
                // Balance on charge station
                Logger.getInstance().recordOutput("Auto/Routine", "Balance on charge station");
                return routines.get("Balance on charge station");
            }
        }
        else if (chargeStation.getSelected().equals("DriveOut")) {
            if (placeGamePiece.getSelected()) {
                // Place on high node and drive out
                Logger.getInstance().recordOutput("Auto/Routine", gamePiece + " place on high node and drive out");
                return routines.get(gamePiece + " place on high node and drive out");
            }
            else {
                // Drive out
                Logger.getInstance().recordOutput("Auto/Routine", "Drive out");
                return routines.get("Drive out");
            }
        }
        else {
            if (placeGamePiece.getSelected()) {
                // Place on high node and do not move
                Logger.getInstance().recordOutput("Auto/Routine", gamePiece + " place on high node and do nothing");
                return routines.get(gamePiece + " place on high node and do not move");
            }
            else {
                // Do nothing
                Logger.getInstance().recordOutput("Auto/Routine", "Do nothing");
                return routines.get("Do nothing");
            }
        }
    }
}
