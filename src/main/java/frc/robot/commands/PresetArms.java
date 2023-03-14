package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;
import frc.robot.Constants;

public class PresetArms extends CommandBase {
    private Arms arms;
    private String position;
    
    public PresetArms(Arms arms, String position) {
        this.arms = arms;
        this.position = position;
    }

    @Override 
    public void initialize() {}

    @Override 
    public void execute() {
        //move to the preset point
        if (position == "high node") {
            //set to high node positon
            arms.setArm(Constants.kHighIntakeSetpoints[0], Constants.kHighIntakeSetpoints[1]);
        } else if (position == "mid node") {
            //set to mid node position
            arms.setArm(Constants.kMidNodeSetpoints[0], Constants.kMidNodeSetpoints[1]);
        } else if (position == "hybrid node") {
            //set to hybrid node position
            arms.setArm(Constants.kHybridNodeSetpoints[0], Constants.kHybridNodeSetpoints[1]);
        } else if (position == "high intake") {
            //set to high intake position
            arms.setArm(Constants.kHighIntakeSetpoints[0], Constants.kHighIntakeSetpoints[1]);
        } else if (position == "ground intake") {
            //set to ground intake position
            arms.setArm(Constants.kGroundIntakeSetpoints[0], Constants.kGroundIntakeSetpoints[1]);
        } else if (position == "starting") {
            //set position to starting position
            arms.setArm(Constants.kStartingSetpoints[0], Constants.kStartingSetpoints[1]);
        } else if (position == "taxi") {
            //set position to taxi position
            arms.setArm(Constants.kTaxiSetpoints[0], Constants.kTaxiSetpoints[1]);
        }
    }
}