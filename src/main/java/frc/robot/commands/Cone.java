package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Cone extends CommandBase {
    private Intake intake;
    
    public Cone(Intake intake) {
        this.intake = intake;
    }

    @Override 
    public void initialize() {
        intake.pickupCone();
    }

}