package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    CANSparkMax intake;
    double intakeSpeed;

    public Intake() {
        // TODO: find ID
        // intake = new CANSparkMax(98, MotorType.kBrushless);
        // intakeSpeed = 4.0;
    }

    @Override
    public void periodic() { 
        
    }

    public void pickupCone(double intakeSpeed) {
        // intake.set(intakeSpeed);
    }

    public void pickupCube(double intakeSpeed) {
        //TODO: Find out which motor is negative
        // intake.set(-1 * intakeSpeed);
    }

    public double getSpd() {
        // return intakeSpeed;
        return 0.0;
    }

}
