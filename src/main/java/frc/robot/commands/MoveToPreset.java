package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import edu.wpi.first.math.util.Units;

public class MoveToPreset extends CommandBase {
    private Arms arms;
    private double[] setpoints;
    private double shoulderSetpoint;
    private double elbowSetpoint;

    private boolean shoulderSet = false;
    private boolean elbowSet = false;

    private double[] speeds;
    
    public MoveToPreset(Arms arms) {
        this.arms = arms;

        addRequirements(arms);
    }

    @Override 
    public void initialize() {}

    @Override 
    public void execute() {
        //TODO figure out which goes first, shoulder or elbow
        //move to the preset point
        if (arms.idle()) {
            arms.stop();
        }
        else {
            setpoints = arms.getPreset();

            speeds = arms.calculateMotorSpeeds(setpoints[0], setpoints[1]);
            arms.runShoulder(speeds[0]);
            arms.runElbow(speeds[1]);
        }    
    }

    @Override
    public void end(boolean interuppted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}