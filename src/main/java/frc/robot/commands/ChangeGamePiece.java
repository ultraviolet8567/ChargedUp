package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.GamePiece;

public class ChangeGamePiece extends CommandBase {
    private GamePiece gamePiece;

    public ChangeGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
    }

    @Override 
    public void initialize() { 
        Robot.setGamePiece(gamePiece);

        String gamePieceString = gamePiece == GamePiece.CONE ? "Cone" : "Cube";
        Logger.getInstance().recordOutput("GamePiece", gamePieceString);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}