package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arms;

public class Stop extends CommandBase {
    private Swerve swerve;
    private Intake intake;
    private Arms arms;

    public Stop(Swerve swerve, Intake intake, Arms arms) {
        this.swerve = swerve;
        this.intake = intake;
        this.arms = arms;
    }

    @Override
    public void initialize() {
        swerve.stopModules();
        intake.stop();
        arms.stop();
    }
}
