package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arms;

public class MoveToPreset extends CommandBase {
    private Arms arms;
    private double shoulderSetpoint, elbowSetpoint;
    private SlewRateLimiter shoulderLimiter, elbowLimiter;
    private double[] setpoints;
    
    public MoveToPreset(Arms arms) {
        this.arms = arms;
        
        shoulderLimiter = new SlewRateLimiter(ArmConstants.kMaxShoulderAcceleration.get());
        elbowLimiter = new SlewRateLimiter(ArmConstants.kMaxElbowAcceleration.get());

        addRequirements(arms);
    }

    @Override 
    public void initialize() { }

    @Override 
    public void execute() {
        // TODO: Determine the optimal order of movement to prevent hitting edge of tensioner
        setpoints = arms.getPreset();
        shoulderSetpoint = setpoints[0];
        elbowSetpoint = setpoints[1];
        
        if (arms.idle()) {
            arms.stop();
        }
        else {
            double[] speeds = arms.calculateMotorSpeeds(shoulderSetpoint, elbowSetpoint);

            double shoulderSpeed = speeds[0];
            double elbowSpeed = speeds[1];

            shoulderSpeed = shoulderLimiter.calculate(shoulderSpeed) * ArmConstants.kMaxShoulderSpeed.get();
            elbowSpeed = elbowLimiter.calculate(elbowSpeed) * ArmConstants.kMaxElbowSpeed.get();
            
            Logger.getInstance().recordOutput("SpeedElbow", speeds[1]);

            arms.runShoulder(speeds[0]);
            arms.runElbow(speeds[1]);
        }    
    }

    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}