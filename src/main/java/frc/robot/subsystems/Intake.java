package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.subsystems.Lights.GamePiece;

public class Intake extends SubsystemBase {
    CANSparkMax intake;

    public Intake() {
        intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
        intake.enableVoltageCompensation(12.0);
        intake.setSmartCurrentLimit(40);
        intake.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Setpoints/Intake", intake.get());
        Logger.getInstance().recordOutput("Measured/Intake", intake.getEncoder().getVelocity());
    }

    public void pickup(GamePiece gamePiece) {
        switch (gamePiece) {
            case CONE:
                intake.set(-ArmConstants.intakeSpeed.get());
                break;
            case CUBE:
                intake.set(ArmConstants.intakeSpeed.get());
                break;
            default:
                break;
        }
    }

    public void drop(GamePiece gamePiece) {
        switch (gamePiece) {
            case CONE:
                intake.set(ArmConstants.intakeSpeed.get());
                break;
            case CUBE:
                intake.set(-ArmConstants.intakeSpeed.get());
                break;
            default:
                break;
        }
    }

    public void stop() {
        intake.stopMotor();
    }
}
