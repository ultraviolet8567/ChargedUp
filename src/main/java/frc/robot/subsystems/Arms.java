package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.commands.StopShoulder;

public class Arms extends SubsystemBase {
    
    private final CANSparkMax shoulder;
    private final CANSparkMax elbow;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);
    }

    @Override
    public void periodic() {
        System.out.println(shoulderDeg());
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
        return (shoulderEncoder.getAbsolutePosition() - Constants.kShoulderOffset) * 360;
    }

    // Should use the REV Through-Bore Encoder for this rather than the SparkMax internal encoder
    //TODO: find the offset of the elbow absolute encoder
    public double elbowDeg() {
        return (elbowEncoder.getAbsolutePosition() - elbowEncoder.getPositionOffset()) * 360;
    }

    public void setArm(int shoulderDegree, int elbowDegree) {
        //shoulder presetting
        if (shoulderDegree > shoulderDeg()){
            while (shoulderDeg() > shoulderDegree) {
                runShoulderForward();
            }
        } else if (shoulderDegree < shoulderDeg()){
            while (shoulderDeg() < shoulderDegree){
                runShoulderBackward();
            }
        }

        //elbow presetting
        if (elbowDegree > elbowDeg()){
            while (elbowDeg() > elbowDegree) {
                runElbowForward();
            }
        } else if (elbowDegree < elbowDeg()){
            while (elbowDeg() < elbowDegree){
                runElbowBackward();
            }
        }
                
    }

    //check if the arm is past the limit of -132 degrees 
    public boolean checkLocationBackward() {
        if (shoulderDeg() <= Constants.kStopArmOne) {
            return false;
        } else {
            return true;
        }
    }

    //check if the arm is past the limit of 168 degrees
    public boolean checkLocationForward() {
        if (shoulderDeg() >= Constants.kStopArmTwo) {
            return false;
        } else {
            return true;
        }
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
