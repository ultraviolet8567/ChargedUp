package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class setCardinalDirection extends CommandBase {
    private Swerve swerve;
    private double desiredAngle;

    public setCardinalDirection(Swerve swerve, double desiredAngle){
        this.desiredAngle = desiredAngle;
        this.swerve = swerve;
    }
    @Override
    public void initialize() {
        swerve.setCardinalDirection(desiredAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
