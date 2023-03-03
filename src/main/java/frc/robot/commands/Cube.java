package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Cube extends CommandBase {
    private Intake intake;
    
    public Cube(Intake intake) {
        this.intake = intake;
    }

    @Override 
    
    public void initialize() {
        intake.pickupCube(); 
    }
}