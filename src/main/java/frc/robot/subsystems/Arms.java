package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.Lights.GamePiece;

public class Arms extends SubsystemBase {
    public final CANSparkMax shoulder, elbow;
    
    private final DutyCycleEncoder shoulderEncoder, elbowEncoder;
    private final ProfiledPIDController shoulderPidController, elbowPidController;

    private Preset presetValue;

    // Motion profile stuff
    private final Constraints shoulderConstraints;
    private final Constraints elbowConstraints;

    public Arms() {
        shoulder = new CANSparkMax(CAN.kShoulderPort, MotorType.kBrushless);
        shoulder.enableVoltageCompensation(12.0);
        shoulder.setSmartCurrentLimit(40);
        shoulder.setIdleMode(IdleMode.kBrake);

        elbow = new CANSparkMax(CAN.kElbowPort, MotorType.kBrushless);
        elbow.enableVoltageCompensation(12.0);
        elbow.setSmartCurrentLimit(40);
        elbow.setInverted(true);
        elbow.setIdleMode(IdleMode.kBrake);

        shoulderEncoder = new DutyCycleEncoder(ArmConstants.kShoulderEncoderPort);
        elbowEncoder = new DutyCycleEncoder(ArmConstants.kElbowEncoderPort);

        // Trapezoidal Motion stuff
        shoulderConstraints = new Constraints(ArmConstants.kMaxShoulderSpeed.get(), ArmConstants.kMaxShoulderAcceleration.get());
        elbowConstraints = new Constraints(ArmConstants.kMaxShoulderSpeed.get(), ArmConstants.kMaxShoulderAcceleration.get());

        // Profiled PID controllers also limit the output to stay within the maximum speed and acceleration constraints
        // This is referred to as Trapezoidal Motion Profiling (because if you graph the velocity or acceleration
        // it creates a trapezoid as the PID controller goes steadily up to the maximum and stays there)
        shoulderPidController = new ProfiledPIDController(ArmConstants.kPShoulder.get(), ArmConstants.kIShoulder.get(), ArmConstants.kDShoulder.get(),
            shoulderConstraints);
        shoulderPidController.setTolerance(ArmConstants.kShoulderPidTolerance);

        elbowPidController = new ProfiledPIDController(ArmConstants.kPElbow.get(), ArmConstants.kIElbow.get(), ArmConstants.kDElbow.get(),
            elbowConstraints);
        elbowPidController.setTolerance(ArmConstants.kElbowPidTolerance);
        
        presetValue = Preset.TAXI;
        resetPIDControllers();
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("Preset/Shoulder", getPreset()[0]);
        Logger.getInstance().recordOutput("Preset/Elbow", getPreset()[1]);

        Logger.getInstance().recordOutput("AbsoluteEncoders/Shoulder", shoulderAngle());
        Logger.getInstance().recordOutput("AbsoluteEncoders/Elbow", elbowAngle());

        Logger.getInstance().recordOutput("Setpoints/Shoulder", shoulder.get());
        Logger.getInstance().recordOutput("Setpoints/Elbow", elbow.get());

        Logger.getInstance().recordOutput("Measured/Shoulder", shoulder.getEncoder().getVelocity());
        Logger.getInstance().recordOutput("Measured/Elbow", elbow.getEncoder().getVelocity());
        
        Logger.getInstance().recordOutput("PIDError/Shoulder", shoulderPidController.getPositionError());
        Logger.getInstance().recordOutput("PIDError/Elbow", elbowPidController.getPositionError());
    }

    public void runShoulder(double speed) {
        shoulder.set(speed);
    }
    
    public void runElbow(double speed) {
        elbow.set(speed);
    }

    public double shoulderAngle() {
        double angle = shoulderEncoder.getAbsolutePosition();
        angle *= 2 * Math.PI;
        angle = translateEncoderAngle(angle, ArmConstants.kShoulderEncoderOffset);
        return angle;
    }

    public double elbowAngle() {
        double angle = elbowEncoder.getAbsolutePosition();
        angle *= 2 * Math.PI;
        angle = translateEncoderAngle(angle, ArmConstants.kElbowEncoderOffset);
        return angle;
    }

    // Translates the absolute encoder readings to make future calculations easier
    public double translateEncoderAngle(double angle, double encoderTranslation) {
        angle = (angle + encoderTranslation) % (2 * Math.PI);
        boolean makeNegative = (angle >= Math.PI / 2 && angle <= 3 * Math.PI / 2);
        
        angle = (angle + Math.PI / 2) % Math.PI;
        if (makeNegative)
            angle -= Math.PI;

        return angle;
    }
    
    public double[] calculateMotorSpeeds(double shoulderSetpoint, double elbowSetpoint) {
        double shoulderSpeed = shoulderPidController.calculate(shoulderAngle(), shoulderSetpoint);
        double elbowSpeed = elbowPidController.calculate(elbowAngle(), elbowSetpoint);
        shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.kMaxShoulderSpeedPercentage, ArmConstants.kMaxShoulderSpeedPercentage);
        elbowSpeed = MathUtil.clamp(elbowSpeed, -ArmConstants.kMaxElbowSpeedPercentage, ArmConstants.kMaxElbowSpeedPercentage);

        if (shoulderPidController.atSetpoint()) {
            // totalShoulderVoltage = 0;
            shoulderSpeed = 0;
        }
        if (elbowPidController.atSetpoint()) {
            // totalElbowVoltage = 0;
            elbowSpeed = 0;
        }

        Logger.getInstance().recordOutput("ArmSpeeds/ShoulderSpeed", shoulderSpeed);
        Logger.getInstance().recordOutput("ArmSpeeds/ElbowSpeed", elbowSpeed);

        return new double[] {shoulderSpeed, elbowSpeed};
    }
    
    // Check if the shoulder can be moved
    public boolean shoulderMovable(double shoulderSpeed) {
        return !(shoulderPastBackLimit() && shoulderSpeed < 0) && !(shoulderPastFrontLimit() && shoulderSpeed > 0);
    }

    // Check if the elbow can be moved
    public boolean elbowMovable(double elbowSpeed) {
        return !(elbowPastBackLimit() && elbowSpeed < 0) && !(elbowPastFrontLimit() && elbowSpeed > 0);
    }

    // Check if the elbow can be moved in preset mode
    public boolean elbowPresetMovable(double shoulderSpeed) {
        return !(shoulderAngle() < -Math.PI / 6 && shoulderSpeed >= 0);
    }

    // Check if the bicep is within the operable range
    public boolean shoulderWithinRange() {
        return ArmConstants.kShoulderBackLimit < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderFrontLimit;
    }

    // Check if the forearm is within the operable range
    public boolean elbowWithinRange() {
        return ArmConstants.kElbowBackLimit < elbowAngle() && elbowAngle() < ArmConstants.kElbowFrontLimit;
    }

    // Check if the bicep has rotated too far backward (and could hit the superstructure)
    public boolean shoulderPastBackLimit() {
        return ArmConstants.kShoulderBackMechanicalStop < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderBackLimit;
    }
    
    // Check if the bicep has rotated too far forward (and could hit the bumpers)
    public boolean shoulderPastFrontLimit() {
        return ArmConstants.kShoulderFrontLimit < shoulderAngle() && shoulderAngle() < ArmConstants.kShoulderFrontMechanicalStop;
    }

    // Check if the forearm has rotated too far backward and could hit the top of the bicep
    public boolean elbowPastBackLimit() {
        return ArmConstants.kElbowBackMechanicalStop < elbowAngle() && elbowAngle() < ArmConstants.kElbowBackLimit;
    }

    // Check if the forearm has rotated too far forward and could hit the bottom of the bicep
    public boolean elbowPastFrontLimit() {
        return ArmConstants.kElbowFrontLimit < elbowAngle() && elbowAngle() < ArmConstants.kElbowFrontMechanicalStop;
    }

    public double[] getPreset() {
        switch (presetValue) {
            case HIGH_NODE:
                return Lights.getInstance().gamePiece.equals(GamePiece.CONE) ? ArmConstants.kHighNodeConeSetpoints : ArmConstants.kHighNodeCubeSetpoints;
            case MID_NODE:
                return Lights.getInstance().gamePiece.equals(GamePiece.CONE) ? ArmConstants.kMidNodeConeSetpoints : ArmConstants.kMidNodeCubeSetpoints;
            case HYBRID_NODE:
                return Lights.getInstance().gamePiece.equals(GamePiece.CONE) ? ArmConstants.kHybridNodeConeSetpoints : ArmConstants.kHybridNodeCubeSetpoints;
            case SUBSTATION_INTAKE:
                return Lights.getInstance().gamePiece.equals(GamePiece.CONE) ? ArmConstants.kSubstationIntakeConeSetpoints : ArmConstants.kSubstationIntakeCubeSetpoints;
            case GROUND_INTAKE:
                return Lights.getInstance().gamePiece.equals(GamePiece.CONE) ? ArmConstants.kGroundIntakeConeSetpoints : ArmConstants.kGroundIntakeCubeSetpoints;
            case START:
                return ArmConstants.kStartSetpoints;
            case TAXI:
            case IDLE:
            default:
                return ArmConstants.kTaxiSetpoints;
        } 
    }

    public void setPresetValue(Preset preset) {
        presetValue = preset;

        // The PID controllers need to be reset any time we switch back to PID from open loop control (manual)
        if (preset == Preset.MANUAL_OVERRIDE)
            resetPIDControllers();
    }

    public Preset getPresetValue() {
        return presetValue;
    }

    public boolean idle() {
        return presetValue == Preset.IDLE;
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

    public void resetPIDControllers() {
        shoulderPidController.reset(shoulderAngle());
        elbowPidController.reset(elbowAngle());
    }
}