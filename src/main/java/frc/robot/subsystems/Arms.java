package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.Preset;
import frc.robot.Constants;
import frc.robot.Robot;

public class Arms extends SubsystemBase {
    public final CANSparkMax shoulder, elbow;
    
    private final DutyCycleEncoder shoulderEncoder, elbowEncoder;
    private final ProfiledPIDController shoulderPidController, elbowPidController;

    private Preset presetValue;

    // Motion profile stuff
    private final Constraints shoulderConstraints;
    private final Constraints elbowConstraints;
    private State shoulderGoal;
    private State elbowGoal;
    private State shoulderNextPoint;
    private State elbowNextPoint;

    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;
    
    // Feedforward voltages
    private Matrix<N2, N1> voltages;

    // Feedforward vectors
    private Vector<N2> positions;
    private Vector<N2> velocities;
    private Vector<N2> accelerations;

    // Simulator stuff
    private double shoulderSimEncoder, elbowSimEncoder;

    Mechanism2d armsMech;
    MechanismRoot2d armsRoot;
    MechanismLigament2d superstructureMech, shoulderMech, elbowMech;

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
        shoulderGoal = new State(0, 0);
        elbowGoal = new State(0, 0);
        shoulderNextPoint = new State(0, 0);
        elbowNextPoint = new State(0, 0);

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

        // Simulator stuff
        shoulderSimEncoder = 0;
        elbowSimEncoder = 0;

        armsMech = new Mechanism2d(2, 2);   
        armsRoot = armsMech.getRoot("Arms", 1, 0);
        superstructureMech = armsRoot.append(new MechanismLigament2d("Superstructure", Units.inchesToMeters(25), 90));
        shoulderMech = superstructureMech.append(new MechanismLigament2d("Bicep", Units.inchesToMeters(20), shoulderSimEncoder, 5, new Color8Bit(0, 0, 255)));
        elbowMech = shoulderMech.append(new MechanismLigament2d("Forearm", Units.inchesToMeters(16), elbowSimEncoder, 3, new Color8Bit(0, 255, 0)));    
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
        
    @Override
    public void simulationPeriodic() {
        Logger.getInstance().recordOutput("Simulator/AbsoluteEncoders/SimShoulder", shoulderSimEncoder);
        Logger.getInstance().recordOutput("Simulator/AbsoluteEncoders/SimElbow", elbowSimEncoder);
        Logger.getInstance().recordOutput("Simulator/Mechanisms", armsMech);

        shoulderMech.setAngle(shoulderSimEncoder);
        elbowMech.setAngle(elbowSimEncoder);
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

        // Trapezoidal motion stuff
        shoulderGoal.position = shoulderSetpoint;
        elbowGoal.position = elbowSetpoint;

        // Getting previous velocites
        double previousShoulderVel = shoulderNextPoint.velocity;
        double previousElbowVel = elbowNextPoint.velocity;

        // Create a motion profile with the given maximum velocity and maximum acceleration
        // constraints for the NextPoint, the desired goal, and the current point.
        shoulderProfile = new TrapezoidProfile(shoulderConstraints, shoulderGoal, shoulderNextPoint);
        elbowProfile = new TrapezoidProfile(elbowConstraints, elbowGoal, elbowNextPoint);
        
        // Retrieve the profiled NextPoint for the next timestep. This NextPoint moves
        // toward the goal while obeying the constraints.
        shoulderNextPoint = shoulderProfile.calculate(Constants.periodicTime);
        elbowNextPoint = elbowProfile.calculate(Constants.periodicTime);

        velocities.set(0, 0, shoulderNextPoint.velocity);
        velocities.set(1, 0, elbowNextPoint.velocity);

        // Calculating acceleration for arms using the last point's velocity & next point's velocity
        double shoulderAccel = (shoulderNextPoint.velocity - previousShoulderVel) / Constants.periodicTime; 
        double elbowAccel = (elbowNextPoint.velocity - previousElbowVel) / Constants.periodicTime; 

        accelerations.set(0, 0, shoulderAccel);
        accelerations.set(1, 0, elbowAccel);

        // Set positions
        // TODO (FIX POSITIONS bc idk if the current positions based off the abs encoders works rn)
        positions.set(0, 0, shoulderAngle());
        positions.set(1, 0, elbowAngle());

        // Use above values to find feedforward shoulder/elbow voltages
        voltages = feedForward(positions, velocities, accelerations);

        // TODO Combine PID & FeedForward Control 

        // Make sure the elbow turns with the shoulder
        // elbowSpeed += shoulderSpeed * ArmConstants.kArmsToElbow;
        
        // Clamp the speeds between -80% and 80%
        shoulderSpeed = MathUtil.clamp(shoulderSpeed, -ArmConstants.kMaxShoulderSpeedPercentage, ArmConstants.kMaxShoulderSpeedPercentage);
        elbowSpeed = MathUtil.clamp(elbowSpeed, -ArmConstants.kMaxElbowSpeedPercentage, ArmConstants.kMaxElbowSpeedPercentage);

        if (shoulderPidController.atSetpoint())
            shoulderSpeed = 0;
        if (elbowPidController.atSetpoint())
            elbowSpeed = 0;

        return new double[] {shoulderSpeed, elbowSpeed};
    }
    
    // All the feed forward math to get arm voltages
    public Matrix<N2, N1> feedForward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        Matrix<N2,N1> voltages = inverseB().times(M(position).times(acceleration)
            .plus(C(position, velocity).times(velocity))
            .plus(Tg(position))
            .plus((Kb()).times(velocity))
        );
        
        return voltages;
    }

    // Returns the values for the M (interia) matrix
    public Matrix<N2, N2> M(Vector<N2> position) {
        Matrix<N2, N2> M = new Matrix<>(N2.instance, N2.instance);
        
        M.set(0, 0, 
            ArmConstants.kShoulderMass * Math.pow(ArmConstants.kShoulderRadius, 2.0)
            + ArmConstants.kElbowMass * (Math.pow(ArmConstants.kShoulderLength, 2.0) + Math.pow(ArmConstants.kElbowRadius, 2.0))
            + ArmConstants.kShoulderInertia
            + ArmConstants.kElbowInertia
            + 2 * ArmConstants.kElbowMass
                * ArmConstants.kShoulderLength
                * ArmConstants.kElbowRadius
                * Math.cos(positions.get(1, 0))
        );

        M.set(1, 0, 
            ArmConstants.kElbowMass * Math.pow(ArmConstants.kElbowRadius, 2.0)
            + ArmConstants.kElbowInertia
            + ArmConstants.kElbowMass
                * ArmConstants.kShoulderLength
                * ArmConstants.kElbowRadius
                * Math.cos(positions.get(1, 0))
        );

        M.set(0, 1, 
            ArmConstants.kElbowMass * Math.pow(ArmConstants.kElbowRadius, 2.0)
            + ArmConstants.kElbowInertia
            + ArmConstants.kElbowMass
                * ArmConstants.kShoulderLength
                * ArmConstants.kElbowRadius
                * Math.cos(positions.get(1, 0))
        );

        M.set(1, 1,
            ArmConstants.kElbowMass * Math.pow(ArmConstants.kElbowRadius, 2.0)
            + ArmConstants.kElbowInertia
        );

        return M;
    }

    // Returns the values for the C (centrifugal & Coriolis) matrix
    public Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        Matrix<N2, N2> C = new Matrix<>(N2.instance, N2.instance);
        
        C.set(0, 0, 
            -ArmConstants.kElbowMass 
            * ArmConstants.kShoulderLength
            * ArmConstants.kElbowRadius
            * Math.sin(positions.get(1, 0))
            * velocities.get(1, 0)
        ); 

        C.set(0, 1,
            ArmConstants.kElbowMass 
            * ArmConstants.kShoulderLength
            * ArmConstants.kElbowRadius
            * Math.sin(positions.get(1, 0))
            * velocities.get(0, 0)
        );

        C.set(1, 0,
            -ArmConstants.kElbowMass 
            * ArmConstants.kShoulderLength
            * ArmConstants.kElbowRadius
            * Math.sin(positions.get(1, 0))
            * (velocities.get(0, 0) + velocities.get(1, 0))
        );

        return C;
    }

    // Returns the values for the Tg (torque applied due to gravity) matrix
    public Matrix<N2, N1> Tg(Vector<N2> position) {
        Matrix<N2, N1> Tg = new Matrix<>(N2.instance, N1.instance);
        
        Tg.set(0, 0, 
            (ArmConstants.kShoulderMass * ArmConstants.kShoulderRadius
                + ArmConstants.kElbowMass * ArmConstants.kShoulderLength)
            * ArmConstants.kGravity * Math.cos(positions.get(0, 0))
            + ArmConstants.kElbowMass
            * ArmConstants.kElbowRadius
            * ArmConstants.kGravity
            * Math.cos(positions.get(0, 0) + positions.get(1, 0))
        ); 

        Tg.set(1, 0,
            ArmConstants.kElbowMass
            * ArmConstants.kElbowRadius
            * ArmConstants.kGravity
            * Math.cos(positions.get(0, 0) + positions.get(1, 0))
        );

        return Tg;
    }

    // Returns the values for the inverse of B (motor torque) matrix
    public Matrix<N2, N2> inverseB() {
        Matrix<N2, N2> B = new Matrix<>(N2.instance, N2.instance);
        Matrix<N2, N2> inverseB = new Matrix<>(N2.instance, N2.instance);
        
        B.set(0, 0, 
            ArmConstants.kShoulderGear
            * (ArmConstants.kShoulderStallTorque / ArmConstants.kShoulderStallCurrent)
            / (ArmConstants.kVoltage / ArmConstants.kShoulderStallCurrent)
        );

        B.set(1, 1,
            ArmConstants.kElbowGear
            * (ArmConstants.kElbowStallTorque / ArmConstants.kElbowStallCurrent)
            / (ArmConstants.kVoltage / ArmConstants.kElbowStallCurrent)
        );

        // Find the inverse of B
        inverseB.set(0, 0, B.get(1, 1));
        inverseB.set(1, 1, B.get(0, 0));

        inverseB.times(1 / (B.get(0, 0) * B.get(1, 1)));

        return inverseB;
    }

    // Returns the values for the Kb (back-emf) matrix
    public Matrix<N2, N2> Kb() {
        Matrix<N2, N2> Kb = new Matrix<>(N2.instance, N2.instance);
        
        Kb.set(0, 0, 
            ArmConstants.kShoulderGear
            * (ArmConstants.kShoulderStallTorque / ArmConstants.kShoulderStallCurrent)
            / (ArmConstants.kShoulderFreeSpeed / ArmConstants.kVoltage)
            / (ArmConstants.kVoltage / ArmConstants.kShoulderStallCurrent)
        );

        Kb.set(1, 1, 
            ArmConstants.kElbowGear
            * (ArmConstants.kElbowStallTorque / ArmConstants.kElbowStallCurrent)
            / (ArmConstants.kElbowFreeSpeed / ArmConstants.kVoltage)
            / (ArmConstants.kVoltage / ArmConstants.kElbowStallCurrent)
        );

        return Kb;
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
                return Robot.getGamePiece().equals(GamePiece.CONE) ? ArmConstants.kHighNodeConeSetpoints : ArmConstants.kHighNodeCubeSetpoints;
            case MID_NODE:
                return Robot.getGamePiece().equals(GamePiece.CONE) ? ArmConstants.kMidNodeConeSetpoints : ArmConstants.kMidNodeCubeSetpoints;
            case HYBRID_NODE:
                return Robot.getGamePiece().equals(GamePiece.CONE) ? ArmConstants.kHybridNodeConeSetpoints : ArmConstants.kHybridNodeCubeSetpoints;
            case SUBSTATION_INTAKE:
                return Robot.getGamePiece().equals(GamePiece.CONE) ? ArmConstants.kSubstationIntakeConeSetpoints : ArmConstants.kSubstationIntakeCubeSetpoints;
            case GROUND_INTAKE:
                return Robot.getGamePiece().equals(GamePiece.CONE) ? ArmConstants.kGroundIntakeConeSetpoints : ArmConstants.kGroundIntakeCubeSetpoints;
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

    
    // public void runShoulderSim(double speed) {
    //     shoulderSimEncoder += speed * 5;
    // }

    // public void runElbowSim(double speed) {
    //     elbowSimEncoder += speed * 5;
    // }

    // public double shoulderAngleSim() {
    //     return shoulderS eimEncoder;
    // }

    // public double elbowAngleSim() {
    //     return elbowSimEncoder;
    // }

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

    // public boolean checkShoulderLocationBackwardSim() {
    //     return !(shoulderAngleSim() >= Units.radiansToDegrees(-ArmConstants.kStopShoulderMid) && shoulderAngleSim() <= Units.radiansToDegrees(ArmConstants.kStopShoulderBackward));
    // }

    // public boolean checkShoulderLocationForwardSim() {
    //     return !(shoulderAngle() <= Units.radiansToDegrees(ArmConstants.kStopShoulderMid) && shoulderAngle() >= Units.radiansToDegrees(ArmConstants.kStopShoulderForward));
    // }

    // public boolean checkElbowLocationBackwardSim() {
    //     return !(elbowAngle() <= Units.radiansToDegrees(ArmConstants.kStopElbowBackward) && elbowAngle() >= Units.radiansToDegrees(-ArmConstants.kStopElbowMid));
    // }

    // public boolean checkElbowLocationForwardSim() {
    //     return !(elbowAngle() >= Units.radiansToDegrees(ArmConstants.kStopElbowForward) && elbowAngle() <= Units.radiansToDegrees(ArmConstants.kStopElbowMid));
    // }
}