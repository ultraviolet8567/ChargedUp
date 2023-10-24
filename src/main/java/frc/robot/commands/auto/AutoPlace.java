package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class AutoPlace extends CommandBase {
    private Arms arms;
    private Intake intake;
    private Timer timer;

    public AutoPlace(Arms arms, Intake intake) {
        this.arms = arms;
        this.intake = intake;

        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.start();
        arms.setPresetValue(Preset.HIGH_NODE);
    }

    @Override
    public void execute() {
        Logger.getInstance().recordOutput("Auto/Timer", timer.get());

        if (timeEquals(4.5)) {
            intake.drop(Lights.getInstance().gamePiece);
        }
        if (timeEquals(4.75)) {
            arms.setPresetValue(Preset.TAXI);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        arms.setPresetValue(Preset.TAXI);
        intake.stop();
    }

    public boolean isFinished() {
        return timeEquals(6.5);
    }

    public boolean timeEquals(double target) {
        return Math.abs(timer.get() - target) < 0.1;
    }
}
