package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Stop extends CommandBase {
    private Swerve swerve;

    public Stop(Swerve swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.stopModules();
    }
}
