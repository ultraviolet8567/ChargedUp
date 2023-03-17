package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Drop extends CommandBase {
    private Intake intake;
    private String gamePiece;
    
    public Drop(Intake intake, String gamePiece) {
        this.intake = intake;
        this.gamePiece = gamePiece;
    }

    @Override 
    public void initialize() {  
        intake.drop(gamePiece);
    }
}