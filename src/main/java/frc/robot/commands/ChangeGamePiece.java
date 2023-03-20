package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class ChangeGamePiece extends CommandBase {
    private RobotContainer container;
    private String gamePiece;

    public ChangeGamePiece(RobotContainer container, String gamePiece) {
        this.container = container;
        this.gamePiece = gamePiece;
    }

    @Override 
    public void initialize() { 
        container.setGamePiece(gamePiece);
        Logger.getInstance().recordOutput("GamePiece", gamePiece);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}