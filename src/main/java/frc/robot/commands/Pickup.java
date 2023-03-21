package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class Pickup extends CommandBase {
    private Intake intake;
    
    public Pickup(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.pickup(Robot.getGamePiece());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}