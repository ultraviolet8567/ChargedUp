package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.Arms;

public class MoveArms extends CommandBase {
    private Arms arms;
    private ArmManual manual;
    private MoveToPreset preset;
    private Supplier<Double> leftJoystickSupplier, rightJoystickSupplier;

    public MoveArms(Arms arms, Supplier<Double> leftJoystickSupplier, Supplier<Double> rightJoystickSupplier) {
        this.arms = arms;
        this.leftJoystickSupplier = leftJoystickSupplier;
        this.rightJoystickSupplier = rightJoystickSupplier;

        manual = new ArmManual(arms, leftJoystickSupplier, rightJoystickSupplier);
        preset = new MoveToPreset(arms);

        addRequirements(arms);
    }

    @Override
    public void initialize() {
        manual.initialize();
        preset.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (arms.idle()) {
            arms.stop();
        }
        else {
            if (arms.getPresetValue() == Preset.MANUAL_OVERRIDE || Math.abs(leftJoystickSupplier.get()) > 0.2 || Math.abs(rightJoystickSupplier.get()) > 0.2) {
                // Manual arm movement
                manual.execute();
                arms.setPresetValue(Preset.MANUAL_OVERRIDE);
            }
            else {
                // Preset automatic movement
                preset.execute();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}