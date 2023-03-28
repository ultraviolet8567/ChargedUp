package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Swerve;

public class AutoDriveOut extends CommandBase {
    private Swerve swerve;
    private GyroOdometry odometry;
    private Timer timer;

    public AutoDriveOut(Swerve swerve, GyroOdometry odometry) {
        this.swerve = swerve;
        this.odometry = odometry;
    }
  
    @Override 
    public void initialize() {
        timer = new Timer();
        timer.start();

        Logger.getInstance().recordOutput("AutoStatus", "Initialize");
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(-0.3, 0, 0);
        SwerveModuleState[] moduleStatesB = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveModuleState[] moduleStates = { new SwerveModuleState(0.5, new Rotation2d(0.0)), new SwerveModuleState(0.5, new Rotation2d(0.0)),
                                             new SwerveModuleState(0.5, new Rotation2d(0.0)), new SwerveModuleState(0.5, new Rotation2d(0.0)) };
        // swerve.setModuleStates(moduleStates);

        Logger.getInstance().recordOutput("Auto/Timer", timer.get());
        Logger.getInstance().recordOutput("AutoStatus", "Executing");
        Logger.getInstance().recordOutput("Auto/SwerveModuleStates", moduleStates);
        Logger.getInstance().recordOutput("Auto/SwerveModuleStatesB", moduleStatesB);

        // if (timer.get() < 5) {
        //     ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.25, 0, 0, odometry.getRotation2d());
        //     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        //     swerve.setModuleStates(moduleStates);
        // }
        // else if (timer.get() < 6) {
        //     ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0.25, odometry.getRotation2d());
        //     SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        //     swerve.setModuleStates(moduleStates);
        // }
        // else {
        //     swerve.stopModules();
        // }
    }

    public void end(boolean interrupted) {
        Logger.getInstance().recordOutput("Auto/Timer", 0);
        Logger.getInstance().recordOutput("AutoStatus", "Done");
        timer.stop();
        // swerve.stopModules();
    }
}
