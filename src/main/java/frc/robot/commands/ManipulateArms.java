package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arms;

public class ManipulateArms extends CommandBase {
  private Arms arms;
  
  public ManipulateArms(Arms arms) {
    this.arms = arms;
    addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.joint() == "shoulder") {
      if (RobotContainer.direction() == "forward") {
        arms.runShoulderForward();
      } else if (RobotContainer.direction() == "backward") {
        arms.runShoulderBackward();
      }
    } else if (RobotContainer.joint() == "elbow") {
      if (RobotContainer.direction() == "forward") {
        arms.runElbowForward();
      } else if (RobotContainer.direction() == "backward") {
        arms.runElbowBackward();
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
