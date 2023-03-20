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

            shoulderSetpoint = setpoints[0];
            elbowSetpoint = setpoints[1];

            if (Constants.currentMode == Mode.REAL) {
                shoulderSet = (Math.abs(arms.shoulderAngle() - shoulderSetpoint) <= 5 * Math.PI / 600);
                elbowSet = (Math.abs(arms.elbowAngle() - elbowSetpoint) <= 5 * Math.PI / 600);
            } else if (Constants.currentMode == Mode.SIM) {
                shoulderSet = (Math.abs(arms.shoulderAngle() - Units.radiansToDegrees(shoulderSetpoint)) <= 5);
                elbowSet = (Math.abs(arms.elbowAngle() - Units.radiansToDegrees(elbowSetpoint)) <= 5);
            }

            speeds = arms.calculateMotorSpeeds(shoulderSetpoint, elbowSetpoint, !shoulderSet, !elbowSet);
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