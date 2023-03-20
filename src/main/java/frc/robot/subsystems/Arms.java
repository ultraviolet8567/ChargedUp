package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

    public boolean shoulderRunning = false;

    private final PIDController shoulderPidController;
    private final PIDController elbowPidController;

    Mechanism2d armsMech;
    MechanismRoot2d armsRoot;
    MechanismLigament2d superstructureMech;
    MechanismLigament2d shoulderMech;
    MechanismLigament2d elbowMech;

    private double shoulderSimEncoder;
    private double elbowSimEncoder;

    private String presetValue;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);

        shoulderEncoder = new DutyCycleEncoder(Constants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(Constants.kElbowEncoderPort);
        shoulderEncoder.setPositionOffset(Constants.kShoulderOffset);
        elbowEncoder.setPositionOffset(Constants.kElbowOffset);

        shoulderSimEncoder = 0;
        elbowSimEncoder = 0;

        shoulderRunning = false;

        shoulderPidController = new PIDController(Constants.armP.get(), 0, 0);
        shoulderPidController.setTolerance(5 * Math.PI / 600);

        elbowPidController = new PIDController(Constants.armP.get(), 0, 0);
        elbowPidController.setTolerance(5 * Math.PI / 600);

        armsMech = new Mechanism2d(2, 2);   
        armsRoot = armsMech.getRoot("arms", 1, 0);
        superstructureMech = armsRoot.append(new MechanismLigament2d("superstructure", Units.inchesToMeters(25), 90));
        shoulderMech = superstructureMech.append(new MechanismLigament2d("bicep", Units.inchesToMeters(20), shoulderAngle(), 5, new Color8Bit(0, 0, 255)));
        elbowMech = shoulderMech.append(new MechanismLigament2d("forearm", Units.inchesToMeters(16), elbowAngle(), 3, new Color8Bit(0, 255, 0)));
    
        presetValue = "idle";
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("AbsoluteEncoders/Shoulder", shoulderAngle());
        Logger.getInstance().recordOutput("AbsoluteEncoders/Elbow", elbowAngle());

        Logger.getInstance().recordOutput("AbsoluteEncoders/SimShoulder", shoulderSimEncoder);
        Logger.getInstance().recordOutput("AbsoluteEncoders/SimElbow", elbowSimEncoder);

        shoulderMech.setAngle(shoulderSimEncoder);
        elbowMech.setAngle(elbowSimEncoder);

        Logger.getInstance().recordOutput("Mechanisms", armsMech);
    }

    public boolean idle() {
        return presetValue == "idle";
    }

    public void runShoulder(double speed) {
        Logger.getInstance().recordOutput("Speeds/Shoulder", speed);

        if (Constants.currentMode == Mode.REAL) {
            shoulder.set(speed);
        } else if (Constants.currentMode == Mode.SIM) {
            shoulderSimEncoder += speed * 5;
        }    
    }
    
    public void runElbow(double speed) {
        Logger.getInstance().recordOutput("Speeds/Elbow", speed);
    
        if (Constants.currentMode == Mode.REAL) {
            elbow.set(speed);
        } else if (Constants.currentMode == Mode.SIM) {
            elbowSimEncoder += speed * 5;
        }
    }

    // In radians
    public double shoulderAngle() {
        double angle = 0;
        if (Constants.currentMode == Mode.REAL) {
            angle = shoulderEncoder.getAbsolutePosition(); // - shoulderEncoder.getPositionOffset();
            angle *= 2 * Math.PI;
        } else if (Constants.currentMode == Mode.SIM) {
            angle = shoulderSimEncoder;
        }
        return angle;
    }

    // In radians
    public double elbowAngle() {
        double angle = 0;
        if (Constants.currentMode == Mode.REAL) {
            angle = elbowEncoder.getAbsolutePosition(); // - elbowEncoder.getPositionOffset();
            angle *= 2 * Math.PI;
        } else if (Constants.currentMode == Mode.SIM) {
            angle = elbowSimEncoder;
        }
        return angle;
    }

    public double[] calculateMotorSpeeds(double shoulderSetpoint, double elbowSetpoint, boolean movingShoulder, boolean movingElbow) {
        double shoulderSpeed = 0;
        double elbowSpeed = 0;

        if (Constants.currentMode == Mode.REAL) {
            if (movingShoulder) {
                shoulderSpeed += shoulderPidController.calculate(shoulderAngle(), shoulderSetpoint);
                elbowSpeed += shoulderSpeed * Constants.kArmsToElbow;
            } 
            if (movingElbow) {
                elbowSpeed += elbowPidController.calculate(elbowAngle(), elbowSetpoint);
            }

        // SIM mode stuff probably will not work
        } else if (Constants.currentMode == Mode.SIM) {
            if (movingShoulder) {
                if (shoulderAngle() > Units.radiansToDegrees(Constants.kStopShoulderBackward) && shoulderAngle() < Units.radiansToDegrees(Constants.kStopShoulderForward)) {
                    if (shoulderAngle() > Units.radiansToDegrees(shoulderSetpoint)) {
                        shoulderSpeed = -0.2;
                    } else if (shoulderAngle() < Units.radiansToDegrees(shoulderSetpoint)) {
                        shoulderSpeed = 0.2;
                    } else {
                        shoulderSpeed = 0;
                    }

                    if (!movingElbow) {
                        elbowSpeed = -296/322 * shoulderSpeed;
                    }
                } else {
                    shoulderSpeed = 0;
                }
            } 
            if (movingElbow) {
                if (elbowAngle() > Units.radiansToDegrees(Constants.kStopElbowBackward) && elbowAngle() < Units.radiansToDegrees(Constants.kStopElbowForward)) {
                    if (elbowAngle() > Units.radiansToDegrees(elbowSetpoint)) {
                        elbowSpeed = -0.2;
                    } else if (elbowAngle() < Units.radiansToDegrees(elbowSetpoint)) {
                        elbowSpeed = 0.2;
                    } else {
                        elbowSpeed = 0;
                    }

                    if (movingShoulder) {
                        elbowSpeed += -296/322 * shoulderSpeed;
                    } 
                } else {
                    elbowSpeed = 0;
                }
            } else if (!movingElbow && !movingShoulder) {
                elbowSpeed = 0;
                shoulderSpeed = 0;
            }
        }
        

        // Clamp the speeds for safety
        shoulderSpeed = MathUtil.clamp(shoulderSpeed, -Constants.kMaxShoulderSpeed.get(), Constants.kMaxShoulderSpeed.get());
        elbowSpeed = MathUtil.clamp(elbowSpeed, -Constants.kMaxElbowSpeed.get(), Constants.kMaxElbowSpeed.get());

        // Don't move the motors if it's past the boundary
        

        return new double[] {shoulderSpeed, elbowSpeed};
    }

    public double[] getPreset() {
        //TODO: change to starting presets
        double shoulderSetpoint = 0;
        double elbowSetpoint = 0;

        if (presetValue == "high node") {
            //set to high node positon
            shoulderSetpoint = Constants.kHighNodeSetpoints[0];
            elbowSetpoint = Constants.kHighNodeSetpoints[1];
        } else if (presetValue == "mid node") {
            //set to mid node position
            shoulderSetpoint = Constants.kMidNodeSetpoints[0];
            elbowSetpoint = Constants.kMidNodeSetpoints[1];
        } else if (presetValue == "hybrid node") {
            //set to hybrid node position
            shoulderSetpoint = Constants.kHybridNodeSetpoints[0];
            elbowSetpoint = Constants.kHybridNodeSetpoints[1];
        } else if (presetValue == "high intake") {
            //set to high intake position
            shoulderSetpoint = Constants.kHighIntakeSetpoints[0];
            elbowSetpoint = Constants.kHighIntakeSetpoints[1];        
        } else if (presetValue == "ground intake") {
            //set to ground intake position
            shoulderSetpoint = Constants.kGroundIntakeSetpoints[0];
            elbowSetpoint = Constants.kGroundIntakeSetpoints[1];     
        } else if (presetValue == "starting") {
            //set position to starting position
            shoulderSetpoint = Constants.kStartingSetpoints[0];
            elbowSetpoint = Constants.kStartingSetpoints[1];               
        } else if (presetValue == "taxi") {
            //set position to taxi position
            shoulderSetpoint = Constants.kTaxiSetpoints[0];
            elbowSetpoint = Constants.kTaxiSetpoints[1];       
        }

        return new double[] {shoulderSetpoint, elbowSetpoint}; 
    }

    public void setPresetValue(String preset) {
        presetValue = preset;
    }

    //check if the bicep is past the limit of 300 degrees moving backward
    public boolean checkShoulderLocationBackward() {
        if (Constants.currentMode == Mode.REAL) {
            if (shoulderAngle() >= Constants.kStopShoulderMid && shoulderAngle() <= -Constants.kStopShoulderBackward) {
                return false;
            } else {
                return true;
            }   
        } else if (Constants.currentMode == Mode.SIM) {
            if (shoulderAngle() >= Units.radiansToDegrees(-Constants.kStopShoulderMid) && shoulderAngle() <= Units.radiansToDegrees(Constants.kStopShoulderBackward)) {
                return false;
            } else {
                return true;
            } 
        }
        return false;
    }

    //check if the bicep is past the limit of 220 degrees moving forward
    public boolean checkShoulderLocationForward() {
        if (Constants.currentMode == Mode.REAL) {
            if (shoulderAngle() <= Constants.kStopShoulderMid && shoulderAngle() >= Constants.kStopShoulderForward) {
                return false;
            } else {
                return true;
            }
        } else if (Constants.currentMode == Mode.SIM) { 
            if (shoulderAngle() <= Units.radiansToDegrees(Constants.kStopShoulderMid) && shoulderAngle() >= Units.radiansToDegrees(Constants.kStopShoulderForward)) {
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    //check if the forearm is past the limit of 225 degrees moving backward
    public boolean checkElbowLocationBackward() {
        if (Constants.currentMode == Mode.REAL) {
            if (elbowAngle() <= Constants.kStopElbowBackward && elbowAngle() >= -Constants.kStopElbowMid) {
                return false;
            } else {
                return true;
            }
        } else if (Constants.currentMode == Mode.SIM) { 
            if (elbowAngle() <= Units.radiansToDegrees(Constants.kStopElbowBackward) && elbowAngle() >= Units.radiansToDegrees(-Constants.kStopElbowMid)) {
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    //check if the forearm is past the limit of 135 degrees moving forward
    public boolean checkElbowLocationForward() {
        if (Constants.currentMode == Mode.REAL) {
            if (elbowAngle() >= Constants.kStopElbowForward && elbowAngle() <= Constants.kStopElbowMid) {
                return false;
            } else {
                return true;
            }
        } else if (Constants.currentMode == Mode.SIM) { 
            if (elbowAngle() >= Units.radiansToDegrees(Constants.kStopElbowForward) && elbowAngle() <= Units.radiansToDegrees(Constants.kStopElbowMid)) {
                return false;
            } else {
                return true;
            }
        }
        return false;
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
