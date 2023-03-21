package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class StopShoulder extends CommandBase {
    private Arms arms;

    public StopShoulder(Arms arms) {
        this.arms = arms;
    }

    @Override
    public void initialize() {
        arms.stopShoulder();
    }
}
