package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class SetPresetValue extends CommandBase {
    private Arms arms;
    private String presetValue;
    
    public SetPresetValue(Arms arms, String presetValue) {
        this.arms = arms;
        this.presetValue = presetValue;
    }

    @Override 
    public void initialize() {
        arms.setPresetValue(presetValue);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}