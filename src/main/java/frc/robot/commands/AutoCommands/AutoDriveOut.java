package frc.robot.commands.AutoCommands;

import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.RobotContainer;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.subsystems.Swerve;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AutoDriveOut extends CommandBase {
    private Swerve swerve;
    private Timer timer;
    private SwerveTeleOp runSwerve;

    public AutoDriveOut(Swerve swerve) {
        this.swerve = swerve;
    }
  
    @Override 
    public void initialize() {
        timer = new Timer();
        timer.start();

        runSwerve = new SwerveTeleOp(
            swerve,
            () -> 2.0,
            () -> 0.0,
            () -> 0.0,
            () -> false,
            () -> true);
        runSwerve.initialize();
    }

    @Override
    public void execute() {
        
        
        if (timer.get() > 1) { 
            runSwerve.execute();
            // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(2.0, 0.0, 0.0, new Rotation2d(0));
        }
        Logger.getInstance().recordOutput("timer", timer.get());
    }
    public void end(boolean interrupted) {
        // chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, new Rotation2d(0));
        timer.stop();
        runSwerve.end(false);
    }
    public boolean isFinished() {
        return timer.get() >= 15;
    }
    public boolean runsWhenDisabled() {
        return false;
    }
}
