package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.CAN;

public class Intake extends SubsystemBase {
    CANSparkMax intake;
    String gamePiece;

    public Intake() {
        intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
        gamePiece = RobotContainer.getGamePiece();
    }

    @Override
    public void periodic() {}

    public void pickup() {
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
