package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;

public class Pickup extends CommandBase {
    private Intake intake;
    
    public Pickup(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.pickup(Lights.getInstance().gamePiece);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}