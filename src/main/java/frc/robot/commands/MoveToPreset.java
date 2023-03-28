package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class MoveToPreset extends CommandBase {
    private Arms arms;
    
    public MoveToPreset(Arms arms) {
        this.arms = arms;
    }

    @Override 
    public void execute() {
        // TODO: Determine the optimal order of movement to prevent hitting edge of tensioner
        double[] setpoints = arms.getPreset();
        double[] speeds = arms.calculateMotorSpeeds(setpoints[0], setpoints[1]);
        
        double shoulderSpeed = speeds[0];
        double elbowSpeed = speeds[1];

        if (arms.shoulderMovable(shoulderSpeed))
            arms.runShoulder(shoulderSpeed);
        // if (arms.elbowMovable(elbowSpeed) && arms.elbowPresetMovable(shoulderSpeed))
        //     arms.runElbow(elbowSpeed);    
    }

    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}