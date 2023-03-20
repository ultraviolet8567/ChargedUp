package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;

public class Drop extends CommandBase {
    private Intake intake;
    
    public Drop(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        intake.drop(Robot.getGamePiece());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}