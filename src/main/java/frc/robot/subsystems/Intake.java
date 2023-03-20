package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Intake extends SubsystemBase {
    CANSparkMax intake;
    // String gamePiece;

    public Intake() {
        intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Speeds/Intake", intake.get());
    }

    public void pickup(String gamePiece) {
        // TODO: Find out which should be negative (cone or cube)
        switch (gamePiece) {
            case "Cone":
                intake.set(-Constants.intakeSpeed.get());
                break;
            case "Cube":
                intake.set(Constants.intakeSpeed.get());
                break;
            default:
                break;
        }
    }

    public void drop(String gamePiece) {
        // TODO: Find out which should be negative (cone or cube)
        
        switch (gamePiece) {
            case "Cone":
                intake.set(Constants.intakeSpeed.get());
                break;
            case "Cube":
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
