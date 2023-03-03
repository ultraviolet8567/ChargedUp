package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Arms extends SubsystemBase {
    
    CANSparkMax shoulder;
    CANSparkMax elbow;
    RelativeEncoder shoulderEnc;
    RelativeEncoder elbowEnc;

    public Arms() {
        shoulder = new CANSparkMax(14, MotorType.kBrushless);
        elbow = new CANSparkMax(17, MotorType.kBrushless);
        shoulderEnc = shoulder.getEncoder();
        elbowEnc = elbow.getEncoder();
    }

    @Override
    public void periodic() { 
    }

   public void turnShoulder() {
        shoulder.set(Constants.armSpeed);
    }
    
    public void turnElbow() {
        elbow.set(Constants.armSpeed);
    }

    public double shoulderDeg() {
        double degrees = (24.0 * shoulderEnc.getPosition() / 5.0);    
        return degrees;
    }

    public void stop() {
        shoulder.stopMotor();
        elbow.stopMotor();
    }
}
