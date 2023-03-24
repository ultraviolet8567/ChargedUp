package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class MoveArms extends CommandBase {
    private Arms arms;
    private ArmManual manual;
    private MoveToPreset preset;
    private Supplier<Double> leftTriggerSupplier, rightTriggerSupplier;

    public MoveArms(Arms arms, Supplier<Double> leftJoystickSupplier, Supplier<Double> rightJoystickSupplier, Supplier<Double> leftTriggerSupplier, Supplier<Double> rightTriggerSupplier) {
        this.arms = arms;
        this.leftTriggerSupplier = leftTriggerSupplier;
        this.rightTriggerSupplier = rightTriggerSupplier;

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
        if (leftTriggerSupplier.get() > 0.5 && rightTriggerSupplier.get() > 0.5) {
            // Manual arm movement
            manual.execute();
        }
        else {
            // Preset automatic movement
            preset.execute();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}