package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Arms extends SubsystemBase {
    
    private final CANSparkMax shoulder;
    private final CANSparkMax elbow;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;
    private final double shoulderEncoderOffset;
    private final double elbowEncoderOffset;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);

        this.shoulderEncoderOffset = shoulderEncoder.getPositionOffset();
        this.elbowEncoderOffset = elbowEncoder.getPositionOffset();
    }

    @Override
    public void periodic() {
        System.out.println(shoulderEncoder.getAbsolutePosition());
        System.out.println(elbowEncoder.getAbsolutePosition());
    }

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
        return shoulderEncoder.getAbsolutePosition() - shoulderEncoderOffset;
    }

    // Should use the REV Through-Bore Encoder for this rather than the SparkMax internal encoder
    public double elbowDeg() {
        return elbowEncoder.getAbsolutePosition() - elbowEncoderOffset;
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

    public void resetAbsoluteEncoders() {
        shoulderEncoder.reset();
        elbowEncoder.reset();
    }
}
