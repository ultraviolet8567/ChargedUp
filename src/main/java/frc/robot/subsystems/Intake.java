package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GamePiece;

public class Intake extends SubsystemBase {
    CANSparkMax intake;

    public Intake() {
        intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Speeds/Intake", intake.get());
        Logger.getInstance().recordOutput("Encoders/Intake", intake.getEncoder().getVelocity());
    }

    public void pickup(GamePiece gamePiece) {
        switch (gamePiece) {
            case CONE:
                intake.set(-Constants.intakeSpeed.get());
                break;
            case CUBE:
                intake.set(Constants.intakeSpeed.get());
                break;
            default:
                break;
        }
    }

    public void drop(GamePiece gamePiece) {
        switch (gamePiece) {
            case CONE:
                intake.set(Constants.intakeSpeed.get());
                break;
            case CUBE:
                intake.set(-Constants.intakeSpeed.get());
                break;
            default:
                break;
        }
    }

    public void stop() {
        intake.stopMotor();
    }
}
