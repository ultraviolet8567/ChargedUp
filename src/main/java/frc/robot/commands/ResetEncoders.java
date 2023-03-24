package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ResetEncoders extends CommandBase {
    private Swerve swerve;

    public ResetEncoders(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}