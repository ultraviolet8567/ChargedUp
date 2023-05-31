package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class SetCardinalDirection extends CommandBase {
    private Swerve swerve;
    private double desiredAngle;

    public SetCardinalDirection(Swerve swerve, double desiredAngle){
        this.desiredAngle = desiredAngle;
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        swerve.cardinalDirection(desiredAngle);
    }

    @Override
    public void end(boolean interuppted) {
        swerve.disableCardinalDirection();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
