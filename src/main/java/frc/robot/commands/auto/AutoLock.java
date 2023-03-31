package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AutoLock extends CommandBase {
    private Swerve swerve;
    private Timer timer;

    public AutoLock(Swerve swerve) {
        this.swerve = swerve;
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
        return timeEquals(0.5);
    }

    public boolean timeEquals(double target) {
        return Math.abs(timer.get() - target) < 0.1;
    }
}
