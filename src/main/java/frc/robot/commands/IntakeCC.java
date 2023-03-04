package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.RobotContainer;

public class IntakeCC extends CommandBase {
    private Intake intake;
    private String gamePiece;
    
    public IntakeCC(Intake intake) {
        this.intake = intake;
    }

    @Override 
    public void initialize() {
        if (RobotContainer.gamePiece() == "cone") {
            intake.pickupCone();
        } else if (RobotContainer.gamePiece() == "cube") {
            intake.pickupCube();
        }
    }

}