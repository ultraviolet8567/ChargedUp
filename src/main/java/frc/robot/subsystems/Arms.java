package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;

public class Arms extends SubsystemBase {
    
    private final CANSparkMax shoulder;
    private final CANSparkMax elbow;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;
    private double shoulderSpeed;
    private double elbowSpeed;

    public boolean shoulderRunning;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);

        shoulderRunning = false;
    }

    @Override
    public void periodic() {
        // while (-(180 - elbowDeg()) < 30) {
        //      elbow.set(Constants.elbowSpeed.get());
        // }
        System.out.println(shoulderDeg() + "     " + elbowDeg());

        Logger.getInstance().recordOutput("Encoders/Shoulder", shoulderDeg());
        Logger.getInstance().recordOutput("Encoders/Elbow", elbowDeg());
    }

    public void runShoulder(double speed) {
        shoulder.set(speed);
    }
    
    public void runElbow(double speed) {
        elbow.set(speed);
    }

    public double shoulderDeg() {
        double shoulderDegrees = -(shoulderEncoder.getAbsolutePosition() - Constants.kShoulderOffset) * 360;
        if (shoulderDegrees < 0) {
            shoulderDegrees += 360;
        }
        return shoulderDegrees;
    }

    //TODO: find the offset of the elbow absolute encoder
    public double elbowDeg() {
        double elbowDegrees = (elbowEncoder.getAbsolutePosition() - Constants.kElbowOffset) * 360;
        if (elbowDegrees < 0) {
            elbowDegrees += 360;
        }
        return elbowDegrees;
    }

    public void setArm(int shoulderDegree, int elbowDegree) {
        //shoulder presetting
        if (shoulderDegree < Constants.kStopShoulderMid) { // if the goal is less than the 260 value
            if(shoulderDeg() < Constants.kStopShoulderMid && shoulderDeg() > shoulderDegree){
                shoulderSpeed = -Constants.shoulderSpeed.get();
                elbowSpeed = -4/5 * shoulderSpeed;

                runShoulder(shoulderSpeed);
                runElbow(elbowSpeed);
            } else {
                shoulderSpeed = Constants.shoulderSpeed.get();
                elbowSpeed = -4/5 * shoulderSpeed;

                runShoulder(Constants.shoulderSpeed.get());
                runElbow(elbowSpeed);
            }
        }

        else if (shoulderDegree > Constants.kStopShoulderMid) { // if the goal is greater than the 260 value
            if(shoulderDeg() > Constants.kStopShoulderMid && shoulderDeg() < shoulderDegree){
                shoulderSpeed = Constants.shoulderSpeed.get();
                elbowSpeed = -4/5 * shoulderSpeed;

                runShoulder(Constants.shoulderSpeed.get());
                runElbow(elbowSpeed);
            } else {
                shoulderSpeed = -Constants.shoulderSpeed.get();
                elbowSpeed = -4/5 * shoulderSpeed;

                runShoulder(shoulderSpeed);
                runElbow(elbowSpeed);
            }
        } else {
            stopShoulder();
        }
        

        //elbow presetting
        if (elbowDegree < 180) { // if the goal is less than the 260 value
            if(elbowDeg() < 180 && elbowDeg() > elbowDegree){
                runElbow(-Constants.elbowSpeed.get());
            } else {
                runElbow(Constants.elbowSpeed.get());
            }
        }

        else if (elbowDegree > 180) { // if the goal is greater than the 260 value
            if(elbowDeg() > 180 && elbowDeg() < elbowDegree){
                runElbow(Constants.elbowSpeed.get());
            } else {
                runElbow(-Constants.elbowSpeed.get());
            }
        } else {
            stopElbow();
        }
                
    }

    //check if the bicep is past the limit of 300 degrees moving backward
    public boolean checkShoulderLocationBackward() {
        if (shoulderDeg() >= Constants.kStopShoulderMid && shoulderDeg() <= Constants.kStopShoulderBackward) {
            return false;
        } else {
            return true;
        }
    }

    //check if the bicep is past the limit of 220 degrees moving forward
    public boolean checkShoulderLocationForward() {
        if (shoulderDeg() <= Constants.kStopShoulderMid && shoulderDeg() >= Constants.kStopShoulderForward) {
            return false;
        } else {
            return true;
        }
    }

    //check if the forearm is past the limit of 225 degrees moving backward
    public boolean checkElbowLocationBackward() {
        if (elbowDeg() <= Constants.kStopElbowBackward && elbowDeg() >= Constants.kStopElbowMid) {
            return false;
        } else {
            return true;
        }
    }

    //check if the forearm is past the limit of 135 degrees moving forward
    public boolean checkElbowLocationForward() {
        if (elbowDeg() >= Constants.kStopElbowForward && elbowDeg() <= Constants.kStopElbowMid) {
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
