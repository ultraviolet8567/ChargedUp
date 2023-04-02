package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private GyroOdometry gyro;
    private Timer timer;
    private PIDController pid;

    public AutoBalance(Swerve swerve, GyroOdometry gyro) {
        this.swerve = swerve;
        this.gyro = gyro;
        timer = new Timer();

        pid = new PIDController(1, 0, 0);
    }

    @Override
    public void initialize() {
        pid.setTolerance(0.05);

        timer.start();
    }

    @Override
    public void execute() {
        double speed = 0;
        // double speed = pid.calculate(gyro.getRotation3d().getX(), 0) * 10;
        if (Math.abs(gyro.getRotation3d().getX()) < 0.05) {
            speed = 0;
        } else {
            if (gyro.getRotation3d().getX() < 0) {
                speed = -1;
            }
            else {
                speed = 1;
            } 
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        swerve.setModuleStates(moduleStates);
        
        Logger.getInstance().recordOutput("Auto/ModuleStates", moduleStates);
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
        return false;
        // return timeEquals(7);
    }

    public boolean timeEquals(double target) {
        return Math.abs(timer.get() - target) < 0.1;
    }
}
