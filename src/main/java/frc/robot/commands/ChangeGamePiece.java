package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.GamePiece;

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
        if (toggle) {
            gamePiece = gamePiece == GamePiece.CONE ? GamePiece.CUBE : GamePiece.CONE;
        }

        Lights.getInstance().gamePiece = gamePiece;
        Logger.getInstance().recordOutput("GamePiece", gamePiece.toString());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}