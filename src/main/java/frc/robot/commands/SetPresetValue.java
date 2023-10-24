package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.Arms;

public class SetPresetValue extends CommandBase {
    private Arms arms;
    private Preset presetValue;
    
    public SetPresetValue(Arms arms, Preset presetValue) {
        this.arms = arms;
        this.presetValue = presetValue;
    }

    @Override 
    public void initialize() {
        arms.setPresetValue(presetValue);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}