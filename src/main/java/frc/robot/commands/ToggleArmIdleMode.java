package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class ToggleArmIdleMode extends CommandBase {
    private Arms arms;

    public ToggleArmIdleMode(Arms arms) {
        this.arms = arms;
    }

    @Override
    public void initialize() {
        arms.toggleArmIdleMode();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}