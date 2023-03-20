package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class PresetArms extends CommandBase {
    private Arms arms;
    private String position;
    private double shoulderSetpoint;
    private double elbowSetpoint;

    private boolean shoulderSet = false;
    private boolean elbowSet = false;

    private double[] speeds;
    
    public PresetArms(Arms arms, String position) {
        this.arms = arms;
        this.position = position;
    }

    @Override 
    public void initialize() {
        if (position == "high node") {
            //set to high node positon
            shoulderSetpoint = Constants.kHighNodeSetpoints[0];
            elbowSetpoint = Constants.kHighNodeSetpoints[1];
        } else if (position == "mid node") {
            //set to mid node position
            shoulderSetpoint = Constants.kMidNodeSetpoints[0];
            elbowSetpoint = Constants.kMidNodeSetpoints[1];
        } else if (position == "hybrid node") {
            //set to hybrid node position
            shoulderSetpoint = Constants.kHybridNodeSetpoints[0];
            elbowSetpoint = Constants.kHybridNodeSetpoints[1];
        } else if (position == "high intake") {
            //set to high intake position
            shoulderSetpoint = Constants.kHighIntakeSetpoints[0];
            elbowSetpoint = Constants.kHighIntakeSetpoints[1];        
        } else if (position == "ground intake") {
            //set to ground intake position
            shoulderSetpoint = Constants.kGroundIntakeSetpoints[0];
            elbowSetpoint = Constants.kGroundIntakeSetpoints[1];     
        } else if (position == "starting") {
            //set position to starting position
            shoulderSetpoint = Constants.kStartingSetpoints[0];
            elbowSetpoint = Constants.kStartingSetpoints[1];               
        } else if (position == "taxi") {
            //set position to taxi position
            shoulderSetpoint = Constants.kTaxiSetpoints[0];
            elbowSetpoint = Constants.kTaxiSetpoints[1];       
        }
    }

    @Override 
    public void execute() {
        //TODO figure out which goes first, shoulder or elbow
        //move to the preset point
        if (Constants.currentMode == Mode.REAL) {
            if (Math.abs(arms.shoulderRadians() - shoulderSetpoint) >= 0.1) {
                shoulderSet = true;
            } else {
                shoulderSet = false;
            }

            if (Math.abs(arms.elbowRadians() - elbowSetpoint) >= 0.1) {
                elbowSet = true;
            } else {
                elbowSet = false;
            }
        } else if (Constants.currentMode == Mode.SIM) {
            if (Math.abs(arms.shoulderRadians() - shoulderSetpoint) >= 1) {
                shoulderSet = true;
            } else {
                shoulderSet = false;
            }

            if (Math.abs(arms.elbowRadians() - elbowSetpoint) >= 1) {
                elbowSet = true;
            } else {
                elbowSet = false;
            }
        }

        speeds = arms.calculateMotorSpeeds(shoulderSetpoint, elbowSetpoint, shoulderSet, elbowSet);

        System.out.println(speeds[0] + " " + speeds[1]);
        System.out.println(arms.shoulderRadians() + " " + arms.elbowRadians());

        arms.runShoulder(speeds[0]);
        arms.runElbow(speeds[1]);           
    }

    @Override
    public void end(boolean interuppted) {
        arms.stop();
    }

    @Override
    public boolean isFinished() {
        if (shoulderSet && elbowSet) {
            return true;
        }
        
        return false;
    }
}