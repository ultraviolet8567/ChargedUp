package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;

public class Stop extends CommandBase {
    private Swerve swerve;
    private Arms arms;
    private Intake intake;

    public Stop(Swerve swerve, Arms arms, Intake intake) {
        this.swerve = swerve;
        this.arms = arms;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        swerve.stopModules();
        arms.stop();
        intake.stop();
    }
}
