package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Preset;
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
    private final PIDController shoulderPidController;
    private final PIDController elbowPidController;

    private Preset presetValue;

    // Simulator stuff
    private double shoulderSimEncoder;
    private double elbowSimEncoder;

    Mechanism2d armsMech;
    MechanismRoot2d armsRoot;
    MechanismLigament2d superstructureMech;
    MechanismLigament2d shoulderMech;
    MechanismLigament2d elbowMech;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);


        shoulderEncoder = new DutyCycleEncoder(ArmConstants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(ArmConstants.kElbowEncoderPort);

        shoulderPidController = new PIDController(ArmConstants.kPShoulder.get(), 0, 0);
        shoulderPidController.setTolerance(ArmConstants.kShoulderPidTolerance);
        // shoulderPidController.enableContinuousInput(-Math.PI, Math.PI);

        elbowPidController = new PIDController(ArmConstants.kPElbow.get(), 0, 0);
        elbowPidController.setTolerance(ArmConstants.kElbowPidTolerance);
        // elbowPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        presetValue = Preset.IDLE;

        // Simulator stuff
        shoulderSimEncoder = 0;
        elbowSimEncoder = 0;

        armsMech = new Mechanism2d(2, 2);   
        armsRoot = armsMech.getRoot("arms", 1, 0);
        superstructureMech = armsRoot.append(new MechanismLigament2d("Superstructure", Units.inchesToMeters(25), 90));
        shoulderMech = superstructureMech.append(new MechanismLigament2d("Bicep", Units.inchesToMeters(20), shoulderAngle(), 5, new Color8Bit(0, 0, 255)));
        elbowMech = shoulderMech.append(new MechanismLigament2d("Forearm", Units.inchesToMeters(16), elbowAngle(), 3, new Color8Bit(0, 255, 0)));    
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("AbsoluteEncoders/Shoulder", shoulderAngle());
        Logger.getInstance().recordOutput("AbsoluteEncoders/Elbow", elbowAngle());
        Logger.getInstance().recordOutput("Measured/Shoulder", shoulder.getEncoder().getVelocity());
        Logger.getInstance().recordOutput("Measured/Elbow", elbow.getEncoder().getVelocity());

        Logger.getInstance().recordOutput("Test/ShoulderFront", shoulderPastFrontLimit());
        Logger.getInstance().recordOutput("Test/ShoulderBack", shoulderPastBackLimit());
        Logger.getInstance().recordOutput("Test/ElbowFront", elbowPastFrontLimit());
        Logger.getInstance().recordOutput("Test/ElbowBack", elbowPastBackLimit());
        
        // Simulator stuff
        Logger.getInstance().recordOutput("Simulator/AbsoluteEncoders/SimShoulder", shoulderSimEncoder);
        Logger.getInstance().recordOutput("Simulator/AbsoluteEncoders/SimElbow", elbowSimEncoder);
        Logger.getInstance().recordOutput("Simulator/Mechanisms", armsMech);

        shoulderMech.setAngle(shoulderSimEncoder);
        elbowMech.setAngle(elbowSimEncoder);
    }

    public boolean idle() {
        return presetValue == Preset.IDLE;
    }

    public void runShoulder(double speed) {
        shoulder.set(speed);
    }

    // public void runShoulderSim(double speed) {
    //     shoulderSimEncoder += speed * 5;
    // }
    
    public void runElbow(double speed) {
        elbow.set(speed);
    }

    // public void runElbowSim(double speed) {
    //     elbowSimEncoder += speed * 5;
    // }

    public double shoulderAngle() {
        double angle = shoulderEncoder.getAbsolutePosition();
        angle *= 2 * Math.PI;
        angle = translateEncoderAngle(angle, ArmConstants.kShoulderEncoderTranslation);
        return angle;
    }

    // public double shoulderAngleSim() {
    //     return shoulderS eimEncoder;
    // }

    public double elbowAngle() {
        double angle = elbowEncoder.getAbsolutePosition();
        angle *= 2 * Math.PI;
        angle = translateEncoderAngle(angle, ArmConstants.kElbowEncoderTranslation);
        return angle;
    }

    // public double elbowAngleSim() {
    //     return elbowSimEncoder;
    // }

    // Translates the absolute encoder readings to make future calculations easier
    public double translateEncoderAngle(double angle, double encoderTranslation) {
        boolean makeNeg = false;
        angle = (angle - encoderTranslation) % (2 * Math.PI);
        if (angle >= Math.PI / 2 && angle <= 3 * Math.PI / 2) {
            makeNeg = true;
        }
        angle = (angle + Math.PI / 2) % Math.PI;
        if (makeNeg) {
            angle -= Math.PI;
        }
        return angle;
    }
    
    public double[] calculateMotorSpeeds(double shoulderSetpoint, double elbowSetpoint) {
        double shoulderSpeed = shoulderPidController.calculate(shoulderAngle(), shoulderSetpoint);
        double elbowSpeed = elbowPidController.calculate(elbowAngle(), elbowSetpoint);

        // Make sure the elbow turns with the shoulder
        elbowSpeed += shoulderSpeed * ArmConstants.kArmsToElbow;
        
        // Clamp the speeds between -100% and 100%
        shoulderSpeed = MathUtil.clamp(shoulderSpeed, -1, 1);
        elbowSpeed = MathUtil.clamp(elbowSpeed, -1, 1);

        System.out.println("{ " + shoulderSetpoint + " " + elbowSetpoint + " }");
        System.out.println("** " + shoulderAngle() + " " + elbowAngle() + " **");
        System.out.println(shoulderPidController.getPositionError() + " " + elbowPidController.getPositionError());

        return new double[] {shoulderSpeed, elbowSpeed};
    }

    // This code does not work
    // public double[] calculateMotorSpeedsSim(double shoulderSetpoint, double elbowSetpoint) {
    //     double shoulderSpeed = 0;
    //     double elbowSpeed = 0;

    //     boolean movingShoulder = !(Math.abs(shoulderAngleSim() - Units.radiansToDegrees(shoulderSetpoint)) <= 5);
    //     boolean movingElbow = !(Math.abs(elbowAngleSim() - Units.radiansToDegrees(elbowSetpoint)) <= 5);

    //     if (movingShoulder) {
    //         if (shoulderAngleSim() > Units.radiansToDegrees(ArmConstants.kStopShoulderBackward) && shoulderAngleSim() < Units.radiansToDegrees(ArmConstants.kStopShoulderForward)) {
    //             if (shoulderAngleSim() > Units.radiansToDegrees(shoulderSetpoint)) {
    //                 shoulderSpeed = -0.2;
    //             } else if (shoulderAngleSim() < Units.radiansToDegrees(shoulderSetpoint)) {
    //                 shoulderSpeed = 0.2;
    //             } else {
    //                 shoulderSpeed = 0;
    //             }

    //             if (!movingElbow) {
    //                 elbowSpeed = ArmConstants.kArmsToElbow * shoulderSpeed;
    //             }
    //         } else {
    //             shoulderSpeed = 0;
    //         }
    //     } 
    //     if (movingElbow) {
    //         if (elbowAngleSim() > Units.radiansToDegrees(ArmConstants.kStopElbowBackward) && elbowAngleSim() < Units.radiansToDegrees(ArmConstants.kStopElbowForward)) {
    //             if (elbowAngleSim() > Units.radiansToDegrees(elbowSetpoint)) {
    //                 elbowSpeed = -0.2;
    //             } else if (elbowAngleSim() < Units.radiansToDegrees(elbowSetpoint)) {
    //                 elbowSpeed = 0.2;
    //             } else {
    //                 elbowSpeed = 0;
    //             }

    //             if (movingShoulder) {
    //                 elbowSpeed += ArmConstants.kArmsToElbow * shoulderSpeed;
    //             } 
    //         } else {
    //             elbowSpeed = 0;
    //         }
    //     } else if (!movingElbow && !movingShoulder) {
    //         elbowSpeed = 0;
    //         shoulderSpeed = 0;
    //     }

    //     // Clamp the speeds for safety
    //     shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.kMaxShoulderSpeed.get(), ArmConstants.kMaxShoulderSpeed.get());
    //     elbowSpeed = MathUtil.clamp(elbowSpeed, -ArmConstants.kMaxElbowSpeed.get(), ArmConstants.kMaxElbowSpeed.get());

    //     // TODO: Don't move the motors past the boundary
    //     // Code here

    //     return new double[] {shoulderSpeed, elbowSpeed};   
    // }

    // Check if the bicep is within the operable range
    public boolean shoulderWithinRange() {
        return 0 < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderFrontLimit ||
            ArmConstants.kShoulderBackLimit < shoulderAngle() && shoulderAngle() < 2 * Math.PI;
           
    }

    // Check if the forearm is within the operable range
    // This NEGLECTS chain tensioners, which would lock the sprocket BEFORE this becomes false
    public boolean elbowWithinRange() {
        return 0 < elbowAngle() && elbowAngle() < ArmConstants.kElbowFrontLimit ||
            ArmConstants.kElbowBackLimit < elbowAngle() && elbowAngle() < 2 * Math.PI;
           
    }

    // Check if the bicep has rotated too far backward (and could hit the superstructure)
    public boolean shoulderPastBackLimit() {
        return ArmConstants.kShoulderBackMechanicalStop < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderBackLimit;
    }

    // public boolean checkShoulderLocationBackwardSim() {
    //     return !(shoulderAngleSim() >= Units.radiansToDegrees(-ArmConstants.kStopShoulderMid) && shoulderAngleSim() <= Units.radiansToDegrees(ArmConstants.kStopShoulderBackward));
    // }

    // Check if the bicep has rotated too far forward (and could hit the bumpers)
    public boolean shoulderPastFrontLimit() {
        return ArmConstants.kShoulderFrontLimit < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderFrontMechanicalStop;
    }

    // public boolean checkShoulderLocationForwardSim() {
    //     return !(shoulderAngle() <= Units.radiansToDegrees(ArmConstants.kStopShoulderMid) && shoulderAngle() >= Units.radiansToDegrees(ArmConstants.kStopShoulderForward));
    // }

    // Check if the forearm has rotated too far backward and could hit the top of the bicep
    // This NEGLECTS chain tensioners, which would lock the sprocket BEFORE this becomes true
    public boolean elbowPastBackLimit() {
        return ArmConstants.kElbowBackMechanicalStop < elbowAngle() && elbowAngle() < ArmConstants.kElbowBackLimit;
    }

    // public boolean checkElbowLocationBackwardSim() {
    //     return !(elbowAngle() <= Units.radiansToDegrees(ArmConstants.kStopElbowBackward) && elbowAngle() >= Units.radiansToDegrees(-ArmConstants.kStopElbowMid));
    // }

    // Check if the forearm has rotated too far forward and could hit the bottom of the bicep
    // This NEGLECTS chain tensioners, which would lock the sprocket BEFORE this becomes true
    public boolean elbowPastFrontLimit() {
        return ArmConstants.kElbowFrontLimit < elbowAngle() && elbowAngle() < ArmConstants.kElbowFrontMechanicalStop;
    }

    // public boolean checkElbowLocationForwardSim() {
    //     return !(elbowAngle() >= Units.radiansToDegrees(ArmConstants.kStopElbowForward) && elbowAngle() <= Units.radiansToDegrees(ArmConstants.kStopElbowMid));
    // }

    public double[] getPreset() {
        switch (presetValue) {
            case HIGH_NODE:
                return ArmConstants.kHighNodeSetpoints;
            case MID_NODE:
                return ArmConstants.kMidNodeSetpoints;
            case HYBRID_NODE:
                return ArmConstants.kHybridNodeSetpoints;
            case SUBSTATION_INTAKE:
                return ArmConstants.kSubstationIntakeSetpoints;
            case GROUND_INTAKE:
                return ArmConstants.kGroundIntakeSetpoints;
            case START:
                return ArmConstants.kStartSetpoints;
            case TAXI:
                return ArmConstants.kTaxiSetpoints;
            case IDLE:
            default:
                return ArmConstants.kStartSetpoints;
        } 
    }

    public void setPresetValue(Preset preset) {
        presetValue = preset;
    }

    public void toggleArmIdleMode() {
        if (shoulder.getIdleMode() == IdleMode.kBrake) {
            shoulder.setIdleMode(IdleMode.kCoast);
            elbow.setIdleMode(IdleMode.kCoast);
        }
        else {
            shoulder.setIdleMode(IdleMode.kBrake);
            elbow.setIdleMode(IdleMode.kBrake);
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