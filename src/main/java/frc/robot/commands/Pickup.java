package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

public class Pickup extends CommandBase {
    private Intake intake;
    private RobotContainer container;
    
    public Pickup(Intake intake, RobotContainer container) {
        this.intake = intake;
        this.container = container;
    }

    @Override
    public void execute() {
        intake.pickup(container.getGamePiece());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}