package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Intake extends SubsystemBase {
    CANSparkMax intake;

    public Intake() {
        intake = new CANSparkMax(CAN.kIntakePort, MotorType.kBrushless);
    }

    @Override
    public void periodic() { }

    public void pickupCone() {
        intake.set(Constants.intakeSpeed);
    }

    public void pickupCube () {
        // TODO: Find out which motor is negative
        intake.set(-Constants.intakeSpeed);
    }

    public void stop() {
        intake.stopMotor();
    }
}
