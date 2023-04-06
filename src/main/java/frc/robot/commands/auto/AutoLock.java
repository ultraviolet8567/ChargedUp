package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoLock extends CommandBase {
    private Swerve swerve;
    private Timer timer;

    public AutoLock(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        swerve.lockWheels();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stopModules();
    }

    public boolean isFinished() {
        return Math.abs(timer.get() - 0.5) < 0.1;
    }
}
