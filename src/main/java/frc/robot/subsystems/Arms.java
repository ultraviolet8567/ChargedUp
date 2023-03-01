package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arms extends SubsystemBase {
    
    CANSparkMax shoulder;
    CANSparkMax elbow;
    RelativeEncoder shoulderEnc;

    public Arms() {
        // shoulder = new CANSparkMax(14, MotorType.kBrushless);
        // elbow = new CANSparkMax(17, MotorType.kBrushless);
        // shoulderEnc = shoulder.getEncoder();
        
    }

   public void ShoulderSpeed() {
        // shoulder.set(4.0);
    }
    
    public void elbowMotor() {
        // elbow.set(4.0);
    }

    public double shoulderDeg() {
        double degrees = (24.0 * shoulderEnc.getPosition() / 5.0);    
        return degrees;
    }

    public void Stop() {
        shoulder.stopMotor();
        elbow.stopMotor();
    }
}
