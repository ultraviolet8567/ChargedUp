package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class ResetGyro extends CommandBase {
    private Swerve swerve;

    public ResetGyro(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.resetGyro();
    }
}