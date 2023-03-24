package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arms;

public class MoveToPreset extends CommandBase {
    private Arms arms;
    private SlewRateLimiter shoulderLimiter, elbowLimiter;
    
    public MoveToPreset(Arms arms) {
        this.arms = arms;
        
        shoulderLimiter = new SlewRateLimiter(ArmConstants.kMaxShoulderAcceleration.get());
        elbowLimiter = new SlewRateLimiter(ArmConstants.kMaxElbowAcceleration.get());
    }

    @Override 
    public void execute() {
        // TODO: Determine the optimal order of movement to prevent hitting edge of tensioner
        double[] setpoints = arms.getPreset();
        
        if (arms.idle()) {
            arms.stop();
        }
        else {
            double[] speeds = arms.calculateMotorSpeeds(setpoints[0], setpoints[1]);

            double shoulderSpeed = shoulderLimiter.calculate(speeds[0]) * ArmConstants.kMaxShoulderSpeed.get();
            double elbowSpeed = elbowLimiter.calculate(speeds[1]) * ArmConstants.kMaxElbowSpeed.get();

            arms.runShoulder(shoulderSpeed);
            arms.runElbow(elbowSpeed);
        }    
    }

    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}