package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private GyroOdometry odometry;
    private Timer timer;
    private SwerveTeleOp forward;
    private SwerveTeleOp backward;

    public AutoBalance(Swerve swerve, GyroOdometry odometry) {
        this.swerve = swerve;
        this.odometry = odometry;
    }
  
    @Override 
    public void initialize() {
        timer = new Timer();
        timer.start();

        forward = new SwerveTeleOp(
            swerve,
            odometry,
            () -> 0.25,
            () -> 0.0,
            () -> 0.0,
            () -> false,
            () -> true);
        backward = new SwerveTeleOp(swerve, odometry, () -> -0.25, () -> 0.0, () -> 0.0, () -> false, () -> true);
        forward.initialize();
        backward.initialize();
    }

    @Override
    public void execute() {
        if (timer.get() < 5) { 
            forward.execute();
        }
        else if (timer.get() < 6) {
            backward.execute();
        }
        else {
            swerve.stopModules();
        }
    }
    public void end(boolean interrupted) {
        timer.stop();
        forward.end(false);
        backward.end(false);
    }
}
