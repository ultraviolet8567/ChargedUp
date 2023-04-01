package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.GamePiece;

public class ChangeGamePiece extends CommandBase {
    private GamePiece gamePiece;
    private boolean toggle;

    public ChangeGamePiece(GamePiece gamePiece) {
        this.gamePiece = gamePiece;
        this.toggle = false;
    }

    public ChangeGamePiece(boolean toggle) {
        this.gamePiece = GamePiece.CONE;
        this.toggle = toggle;
    }

    @Override 
    public void initialize() { 
        if (toggle)
            gamePiece = gamePiece == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE;

        Robot.setGamePiece(gamePiece);

        String gamePieceString = gamePiece == GamePiece.CONE ? "Cone" : "Cube";
        Logger.getInstance().recordOutput("GamePiece", gamePieceString);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}