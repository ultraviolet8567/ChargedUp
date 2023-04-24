package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final GyroOdometry gyro;
    private Timer timer;
    private PIDController pid;

    public AutoBalance(Swerve swerve, GyroOdometry gyro) {
        this.swerve = swerve;
        this.gyro = gyro;
        addRequirements(swerve);

        timer = new Timer();

        if (DriverStation.getAlliance() == Alliance.Red) {
            pid = new PIDController(2.75, 0, 0);
        } else {
            pid = new PIDController(2.25, 0, 0);
        }
        pid.setTolerance(0.1);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double speed = pid.calculate(gyro.getRotation3d().getX(), 0);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);
        
        Logger.getInstance().recordOutput("Auto/Timer", timer.get());
        Logger.getInstance().recordOutput("Auto/Speed", speed);
        Logger.getInstance().recordOutput("Auto/PIDError", pid.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.lockWheels();
    }

    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
