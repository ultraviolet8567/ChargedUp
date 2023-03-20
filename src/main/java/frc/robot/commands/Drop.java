package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class Drop extends CommandBase {
    private Intake intake;
    private RobotContainer container;
    
    public Drop(Intake intake, RobotContainer container) {
        this.intake = intake;
        this.container = container;
    }

    @Override
    public void execute() {
        intake.drop(container.getGamePiece());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}