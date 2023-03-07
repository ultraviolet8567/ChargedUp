package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Pickup extends CommandBase {
    private Intake intake;
    
    public Pickup(Intake intake) {
        this.intake = intake;
    }

    @Override 
    public void initialize() {  
        intake.pickup();
    }
}