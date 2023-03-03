package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCC extends CommandBase {
    private Intake intake;
    private String gamePiece;
    
    public IntakeCC(Intake intake, String gamePiece) {
        this.intake = intake;
        this.gamePiece = gamePiece;
    }

    @Override 
    public void initialize() {
        if (gamePiece == "cone") {
            intake.pickupCone();
        } else if (gamePiece == "cube") {
            intake.pickupCube();
        }
    }

}