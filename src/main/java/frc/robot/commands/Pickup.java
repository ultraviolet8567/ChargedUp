package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Pickup extends CommandBase {
    private Intake intake;
    private String gamePiece;
    
    public Pickup(Intake intake, String gamePiece) {
        this.intake = intake;
        this.gamePiece = gamePiece;
    }

    @Override 
    public void initialize() {  
        intake.pickup(gamePiece);
    }
}