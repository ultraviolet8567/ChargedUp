package frc.robot.subsystems;

import java.lang.reflect.GenericSignatureFormatError;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class Arms extends SubsystemBase {
    
    public final CANSparkMax shoulder;
    public final CANSparkMax elbow;
    private final DutyCycleEncoder shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;
    private double shoulderSpeed;
    private double elbowSpeed;

    public boolean shoulderRunning = false;

    private final PIDController armsPidController;

    Mechanism2d armsMech;
    MechanismRoot2d armsRoot;
    MechanismLigament2d superstructureMech;
    MechanismLigament2d shoulderMech;
    MechanismLigament2d elbowMech;

    private double shoulderSimEncoder;
    private double elbowSimEncoder;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);

        shoulderSimEncoder = 0;
        elbowSimEncoder = 0;

        shoulderRunning = false;

        armsPidController = new PIDController(0.05, 0, 0);

        armsMech = new Mechanism2d(2, 3);   
        armsRoot = armsMech.getRoot("arms", 1, 0);
        superstructureMech = armsRoot.append(new MechanismLigament2d("superstructure", Units.inchesToMeters(25), 90));
        shoulderMech = superstructureMech.append(new MechanismLigament2d("bicep", Units.inchesToMeters(20), shoulderRadians(), 5, new Color8Bit(0, 0, 255)));
        elbowMech = shoulderMech.append(new MechanismLigament2d("forearm", Units.inchesToMeters(16), elbowRadians(), 3, new Color8Bit(0, 255, 0)));
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("AbsoluteEncoders/Shoulder", shoulderRadians());
        Logger.getInstance().recordOutput("AbsoluteEncoders/Elbow", elbowRadians());

        Logger.getInstance().recordOutput("AbsoluteEncoders/SimShoulder", shoulderSimEncoder);
        Logger.getInstance().recordOutput("AbsoluteEncoders/SimElbow", elbowSimEncoder);

        shoulderMech.setAngle(shoulderSimEncoder);
        elbowMech.setAngle(elbowSimEncoder);

        Logger.getInstance().recordOutput("Mechanisms", armsMech);
    }

    public void runShoulder(double speed) {
        if (Constants.currentMode == Mode.REAL) {
            shoulder.set(speed);
        } else if (Constants.currentMode == Mode.SIM) {
            shoulderSimEncoder += speed * 5;
        }    
    }
    
    public void runElbow(double speed) {
        if (Constants.currentMode == Mode.REAL) {
            elbow.set(speed);
        } else if (Constants.currentMode == Mode.SIM) {
            elbowSimEncoder += speed * 5;
        }
    }

    public double shoulderRadians() {
        double shoulderDegrees = 0;
        if (Constants.currentMode == Mode.REAL) {
            shoulderDegrees = -(shoulderEncoder.getAbsolutePosition() - Constants.kShoulderOffset) * 2 * Math.PI - (Math.PI);
        } else if (Constants.currentMode == Mode.SIM) {
            shoulderDegrees = shoulderSimEncoder;
        }
        return shoulderDegrees;
    }

    public double elbowRadians() {
        double elbowDegrees = 0;
        if (Constants.currentMode == Mode.SIM) {
            elbowDegrees = (elbowEncoder.getAbsolutePosition() - Constants.kElbowOffset) * 2 * Math.PI - (Math.PI);
        } else if (Constants.currentMode == Mode.SIM) {
            elbowDegrees = elbowSimEncoder;
        }
        return elbowDegrees;
    }

    public double[] calculateMotorSpeeds(double shoulderSetpoint, double elbowSetpoint, boolean movingShoulder, boolean movingElbow) {
        if (Constants.currentMode == Mode.REAL) {
            if (movingShoulder) {
                if (shoulderRadians() > Constants.kStopShoulderBackward && shoulderRadians() < Constants.kStopShoulderForward) {
                    shoulderSpeed = armsPidController.calculate(shoulderRadians(), shoulderSetpoint);

                    if (!movingElbow) {
                        elbowSpeed = -296/322 * shoulderSpeed;
                    }
                } else {
                    shoulderSpeed = 0;
                }
            } 
            if (movingElbow) {
                if (elbowRadians() > Constants.kStopElbowBackward && elbowRadians() < Constants.kStopElbowForward) {
                    elbowSpeed = armsPidController.calculate(elbowRadians(), elbowSetpoint);

                    if (movingShoulder) {
                        elbowSpeed += -296/322 * shoulderSpeed;
                    } 
                } else {
                    elbowSpeed = 0;
                }
            }
        } else if (Constants.currentMode == Mode.SIM) {
            if (movingShoulder) {
                if (shoulderRadians() > Units.radiansToDegrees(Constants.kStopShoulderBackward) && shoulderRadians() < Units.radiansToDegrees(Constants.kStopShoulderForward)) {
                    if (shoulderRadians() > shoulderSetpoint) {
                        shoulderSpeed = -0.5;
                    } else if (shoulderRadians() < shoulderSetpoint) {
                        shoulderSpeed = 0.5;
                    }

                    if (!movingElbow) {
                        elbowSpeed = -296/322 * shoulderSpeed;
                    }
                } else {
                    shoulderSpeed = 0;
                }
            } 
            if (movingElbow) {
                if (elbowRadians() > Units.radiansToDegrees(Constants.kStopElbowBackward) && elbowRadians() < Units.radiansToDegrees(Constants.kStopElbowForward)) {
                    elbowSpeed = armsPidController.calculate(elbowRadians(), elbowSetpoint);
                    if (elbowRadians() > elbowSetpoint) {
                        elbowSpeed = -0.5;
                    } else if (elbowRadians() < elbowSetpoint) {
                        elbowSpeed = 0.5;
                    }

                    if (movingShoulder) {
                        elbowSpeed += -296/322 * shoulderSpeed;
                    } 
                } else {
                    elbowSpeed = 0;
                }
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
