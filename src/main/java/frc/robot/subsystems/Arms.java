package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Arms extends SubsystemBase {
    
    CANSparkMax shoulder;
    CANSparkMax elbow;
    RelativeEncoder shoulderEncoder;
    RelativeEncoder elbowEncoder;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);
    }

    @Override
    public void periodic() {}

    public void runShoulderForward() {
        shoulder.set(Constants.shoulderSpeed.get());
    }
    
    public void runElbowForward() {
        elbow.set(Constants.elbowSpeed.get());
    }

    public void runShoulderBackward() {
        shoulder.set(-Constants.shoulderSpeed.get());
    }
    
    public void runElbowBackward() {
        elbow.set(-Constants.elbowSpeed.get());
    }

    // Should use the REV Through-Bore Encoder for this rather than the SparkMax internal encoder
    public double shoulderDeg() {
        // return 24.0 * shoulderEncoder.getPosition() / 5.0;
        return 0.0;
    }

    // Should use the REV Through-Bore Encoder for this rather than the SparkMax internal encoder
    public double elbowDeg() {
        // return 24.0 * elbowEncoder.getPosition() / 5.0; 
        return 0.0;
    }

    public void stop() {
        shoulder.stopMotor();
        elbow.stopMotor();
    }

    public void stopShoulder() {
        shoulder.stopMotor();
    }

    public void stopElbow() {
        elbow.stopMotor();
    }
}
