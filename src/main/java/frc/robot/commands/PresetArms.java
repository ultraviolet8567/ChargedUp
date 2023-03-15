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
        //TODO figure out which goes first, shoulder or elbow
        //move to the preset point
        if (position == "high node") {
            //set to high node positon
            arms.setShoulder(Constants.kHighIntakeSetpoints[0]);
            arms.setElbow(Constants.kHighIntakeSetpoints[1]);
        } else if (position == "mid node") {
            //set to mid node position
            arms.setShoulder(Constants.kMidNodeSetpoints[0]);
            arms.setElbow(Constants.kMidNodeSetpoints[1]);
        } else if (position == "hybrid node") {
            //set to hybrid node position
            arms.setShoulder(Constants.kHybridNodeSetpoints[0]);
            arms.setElbow(Constants.kHybridNodeSetpoints[1]);
        } else if (position == "high intake") {
            //set to high intake position
            arms.setShoulder(Constants.kHighIntakeSetpoints[0]);
            arms.setElbow(Constants.kHighIntakeSetpoints[1]);        
        } else if (position == "ground intake") {
            //set to ground intake position
            arms.setShoulder(Constants.kGroundIntakeSetpoints[0]);
            arms.setElbow(Constants.kGroundIntakeSetpoints[1]);        
        } else if (position == "starting") {
            //set position to starting position
            arms.setShoulder(Constants.kStartingSetpoints[0]);
            arms.setElbow(Constants.kStartingSetpoints[1]);                
        } else if (position == "taxi") {
            //set position to taxi position
            arms.setShoulder(Constants.kTaxiSetpoints[0]);
            arms.setElbow(Constants.kTaxiSetpoints[1]);        
        }
    }
}