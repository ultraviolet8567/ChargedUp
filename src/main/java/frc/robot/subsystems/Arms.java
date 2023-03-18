package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class Arms extends SubsystemBase {
    
    public final CANSparkMax shoulder;
    public final CANSparkMax elbow;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;
    private double shoulderSpeed;
    private double elbowSpeed;

    public boolean shoulderRunning = false;

    private final PIDController armsPidController;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);

        shoulderRunning = false;

        armsPidController = new PIDController(0.05, 0, 0);
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("AbsoluteEncoders/Shoulder", shoulderRadians());
        Logger.getInstance().recordOutput("AbsoluteEncoders/Elbow", shoulderRadians());
    }

    public void runShoulder(double speed) {
        shoulder.set(speed);
    }
    
    public void runElbow(double speed) {
        elbow.set(speed);
    }

    public double shoulderRadians() {
        double shoulderDegrees = -(shoulderEncoder.getAbsolutePosition() - Constants.kShoulderOffset) * 2 * Math.PI - (Math.PI/3);
        if (shoulderDegrees < 0) {
            shoulderDegrees += 360;
        }
        return shoulderDegrees;
    }

    //TODO: find the offset of the elbow absolute encoder
    public double elbowRadians() {
        double elbowDegrees = (elbowEncoder.getAbsolutePosition() - Constants.kElbowOffset) * 2 * Math.PI - (3 * Math.PI / 4);
        if (elbowDegrees < 0) {
            elbowDegrees += 360;
        }
        return elbowDegrees;
    }

    public double[] calculateMotorSpeeds(int shoulderSetpoint, int elbowSetpoint, boolean movingShoulder, boolean movingElbow) {
        if (movingShoulder) {
            if (shoulderRadians() > 0 && shoulderRadians() < 5 * Math.PI / 3) {
                shoulderSpeed = armsPidController.calculate(shoulderRadians(), shoulderSetpoint);

                if (!movingElbow) {
                    elbowSpeed = -296/322 * shoulderSpeed;
                }
            } else {
                shoulderSpeed = 0;
            }
        } 
        if (movingElbow) {
            if (elbowRadians() > 0 && elbowRadians() < 5 * Math.PI / 3) {
                elbowSpeed = armsPidController.calculate(elbowRadians(), elbowSetpoint);
                if (movingShoulder) {
                    elbowSpeed += -296/322 * shoulderSpeed;
                } 
            } else {
                elbowSpeed = 0;
            }
        }

        //clamp the speeds between -0.5 & 0.5 for safety & testing
        shoulderSpeed = MathUtil.clamp(shoulderSpeed, -0.5, 0.5);
        elbowSpeed = MathUtil.clamp(elbowSpeed, -0.5, 0.5);

        return new double[] {shoulderSpeed, elbowSpeed};
    }

    // public void setShoulder(int setpoint) {
    //     //shoulder presetting
    //     if (setpoint < Constants.kStopShoulderMid) { 
    //         // if the target angle is between 0 and middle of superstructure
            
    //         if (shoulderDeg() < Constants.kStopShoulderMid && shoulderDeg() > setpoint){

    //             // if current angle is between target angle and middle of superstructure
    //             // run shoulder backwards & elbow compensates
    //             shoulderSpeed = -Constants.shoulderSpeed.get();
    //             elbowSpeed = -4/5 * shoulderSpeed;

    //             runShoulder(shoulderSpeed);
    //             // runElbow(elbowSpeed);

    //             if (shoulderDeg() <= setpoint) {
    //                 shoulderSet = true;
    //                 shoulder.stopMotor();
    //             } else {
    //                 shoulderSet = false;
    //             }
    //         } else {
    //             // if the current angle is not between target angle and middle of superstructure
    //             // run shoulder forwards & elbow compensates
    //             shoulderSpeed = Constants.shoulderSpeed.get();
    //             elbowSpeed = -4/5 * shoulderSpeed;

    //             runShoulder(shoulderSpeed);
    //             // runElbow(elbowSpeed);

    //             if (shoulderDeg() >= setpoint && shoulderDeg() <= Constants.kStopShoulderMid) {
    //                 shoulderSet = true;
    //                 shoulder.stopMotor();
    //             } else {
    //                 shoulderSet = false;
    //             }
    //         }
    //     }

    //     else if (setpoint > Constants.kStopShoulderMid) {
    //         // if the target angle is between middle of superstructure and 360

    //         if (shoulderDeg() > Constants.kStopShoulderMid && shoulderDeg() < setpoint){
    //             // if current angle is between target angle and middle of superstructure
    //             // run shoulder forwards & elbow compensates
    //             shoulderSpeed = Constants.shoulderSpeed.get();
    //             elbowSpeed = -4/5 * shoulderSpeed;

    //             runShoulder(Constants.shoulderSpeed.get());
    //             // runElbow(elbowSpeed);

    //             if (shoulderDeg() >= setpoint) {
    //                 shoulderSet = true;
    //                 shoulder.stopMotor();
    //             } else {
    //                 shoulderSet = false;
    //             }
    //         } else {
    //             // if current angle is not between target angle and middle of superstructure
    //             // run shoulder backwards & elbow compensates
    //             shoulderSpeed = -Constants.shoulderSpeed.get();
    //             elbowSpeed = -4/5 * shoulderSpeed;

    //             runShoulder(shoulderSpeed);
    //             // runElbow(elbowSpeed);

    //             if (shoulderDeg() <= setpoint) {
    //                 shoulderSet = true;
    //                 shoulder.stopMotor();
    //             } else {
    //                 shoulderSet = false;
    //             }
    //         }
    //     } else {
    //         stopShoulder();
    //     }        
    // }

    // public void setElbow(int setpoint) {
    //     //elbow presetting
    //     if (setpoint < Constants.kStopElbowMid) { 
    //         // if the target angle is between 0 and 180
            
    //         //run elbow backwards or forwards based on current position
    //         if (elbowDeg() < Constants.kStopElbowMid && elbowDeg() > setpoint) {
    //             runElbow(-Constants.elbowSpeed.get());

    //             if (elbowDeg() <= setpoint) {
    //                 elbowSet = true;
    //                 elbow.stopMotor();
    //             } else {
    //                 elbowSet = false;
    //             }
    //         } else {
    //             runElbow(Constants.elbowSpeed.get());

    //             if (elbowDeg() >= setpoint) {
    //                 elbowSet = true;
    //                 elbow.stopMotor();
    //             } else {
    //                 elbowSet = false;
    //             }
    //         }
    //     }

    //     else if (setpoint > Constants.kStopElbowMid) { 
    //         // if the target angle is greater than 180 

    //         //run elbow backwards or forwards based on current position
    //         if(elbowDeg() > Constants.kStopElbowMid && elbowDeg() < setpoint){
    //             runElbow(Constants.elbowSpeed.get());

    //             if (elbowDeg() >= setpoint) {
    //                 elbowSet = true;
    //                 elbow.stopMotor();
    //             } else {
    //                 elbowSet = false;
    //             }
    //         } else {
    //             runElbow(-Constants.elbowSpeed.get());

    //             if (elbowDeg() <= setpoint) {
    //                 elbowSet = true;
    //                 elbow.stopMotor();
    //             } else {
    //                 elbowSet = false;
    //             }
    //         }
    //     } else {
    //         stopElbow();
    //     }
    // }

    //check if the bicep is past the limit of 300 degrees moving backward
    public boolean checkShoulderLocationBackward() {
        if (shoulderRadians() >= Constants.kStopShoulderMid && shoulderRadians() <= Constants.kStopShoulderBackward) {
            return false;
        } else {
            return true;
        }
    }

    //check if the bicep is past the limit of 220 degrees moving forward
    public boolean checkShoulderLocationForward() {
        if (shoulderRadians() <= Constants.kStopShoulderMid && shoulderRadians() >= Constants.kStopShoulderForward) {
            return false;
        } else {
            return true;
        }
    }

    //check if the forearm is past the limit of 225 degrees moving backward
    public boolean checkElbowLocationBackward() {
        if (elbowRadians() <= Constants.kStopElbowBackward && elbowRadians() >= Constants.kStopElbowMid) {
            return false;
        } else {
            return true;
        }
    }

    //check if the forearm is past the limit of 135 degrees moving forward
    public boolean checkElbowLocationForward() {
        if (elbowRadians() >= Constants.kStopElbowForward && elbowRadians() <= Constants.kStopElbowMid) {
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
