package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class Drop extends CommandBase {
    private Intake intake;
    
    public Drop(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.drop(Lights.getInstance().gamePiece);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}