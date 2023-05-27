package frc.robot.commands;

import org.littletonrobotics.junction.Logger;
import frc.robot.commands.SwerveTeleOp;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class setCardinalDirection extends CommandBase {
    private Swerve swerve;
    private String buttonPressed;
    private int desiredAngle;

    public setCardinalDirection(Swerve swerve, String buttonPressed){
        this.buttonPressed = buttonPressed;
        this.swerve = swerve;
    }
    @Override
    public void initialize() {
        switch(buttonPressed){
            case "Y":
                desiredAngle = 0;
            case "B":
                desiredAngle = 90;
            case "A":
                desiredAngle = 180;
            case "X":
                desiredAngle = 270;
            default:
                desiredAngle = 0;
        }

        swerve.setCardinalDirection(desiredAngle);
    }
}
