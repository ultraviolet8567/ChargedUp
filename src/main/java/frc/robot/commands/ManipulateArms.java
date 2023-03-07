package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arms;

public class ManipulateArms extends CommandBase {
  private Arms arms;
  private String joint;
  private String direction;
  
  public ManipulateArms(Arms arms, String joint, String direction) {
    this.arms = arms;
    addRequirements(arms);

    this.joint = joint;
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joint == "shoulder") {
      if (direction == "forward" && arms.checkLocationForward()) {
        arms.runShoulderForward();
      } else if (direction == "backward" && arms.checkLocationBackward()) {
        arms.runShoulderBackward(); 
      } else {
        System.out.println("hello");
        arms.stopShoulder();
      }
    } else if (joint == "elbow") {
      if (direction == "forward") {
        arms.runElbowForward();
      } else if (direction == "backward") {
        arms.runElbowBackward();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
