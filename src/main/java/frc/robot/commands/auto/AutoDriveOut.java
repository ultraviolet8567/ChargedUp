package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.odometry.GyroOdometry;
import frc.robot.odometry.Odometry;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveOut extends CommandBase {
    private Swerve swerve;
    private GyroOdometry odometry;
    private Timer timer;
    private SwerveTeleOp runSwerve;

    public AutoDriveOut(Swerve swerve, GyroOdometry odometry) {
        this.swerve = swerve;
        this.odometry = odometry;
    }
  
    @Override 
    public void initialize() {
        timer = new Timer();
        timer.start();

        runSwerve = new SwerveTeleOp(
            swerve,
            odometry,
            () -> 0.25,
            () -> 0.0,
            () -> 0.0,
            () -> false,
            () -> true);
        runSwerve.initialize();
    }

    @Override
    public void execute() {
        if (timer.get() < 5) { 
            runSwerve.execute();
        }
        else {
            swerve.stopModules();
        }
    }
    public void end(boolean interrupted) {
        timer.stop();
        runSwerve.end(false);
    }
}
