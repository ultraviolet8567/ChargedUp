package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Arms;

public class ManipulateArms extends CommandBase {
  private Arms arms;
  private String joint;
  private String direction;
  private double elbowSpeed;
  private double shoulderSpeed;
  private boolean stop;
  private Joystick controller;
  
  public ManipulateArms(Arms arms, String joint, String direction) {
    this.arms = arms;
    addRequirements(arms);

    this.joint = joint;
    this.direction = direction;

    elbowSpeed = 0;
    shoulderSpeed = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = RobotContainer.driverJoystick;

    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println(joint + " " + direction);
      if (joint == "shoulder") {
        arms.shoulderRunning = true;
        
        //run shoulder forward
        if (direction == "forward" && arms.checkShoulderLocationForward()) {
          
          //determine speeds for shoulder and elbow
          shoulderSpeed = Constants.kMaxShoulderSpeed.get();
          elbowSpeed = -4/5 * shoulderSpeed;

          //set shoulder and elbow to determined speeds
          arms.runShoulder(shoulderSpeed);
          arms.runElbow(elbowSpeed);

          if (!controller.getRawButton(XboxController.Button.kX.value)) {
            stop = true;
          }

        //run shoulder backward
        } else if (direction == "backward" && arms.checkShoulderLocationBackward()) {
          
          //determine speeds for shoulder and elbow
          shoulderSpeed = -Constants.kMaxShoulderSpeed.get();
          elbowSpeed = -4/5 * shoulderSpeed;

          //set shoulder and elbow to determined speeds
          arms.runShoulder(shoulderSpeed);
          arms.runElbow(elbowSpeed);

          if (!controller.getRawButton(XboxController.Button.kY.value)) {
            stop = true;
          }

        //in case something goes wrong
        } else {
          arms.stopShoulder();
        }
      } else if (joint == "elbow") {
        
        //run elbow forward
        if (direction == "forward" && arms.checkElbowLocationForward()) {
          
          //determine and set elbow speed if shoulder is running
          if (arms.shoulderRunning) {
            elbowSpeed = (-4/5 * shoulderSpeed) + Constants.kMaxElbowSpeed.get();
            arms.runElbow(elbowSpeed);

          //determine and set elbow speed if shoulder is NOT running
          } else {
            elbowSpeed = Constants.kMaxElbowSpeed.get();
            arms.runElbow(elbowSpeed);
          }

          if (!controller.getRawButton(XboxController.Button.kB.value)) {
            stop = true;
          }

          //run elbow backward
        } else if (direction == "backward" && arms.checkElbowLocationBackward()) {
          
          //determine and set elbow speed if shoulder is running
          if (arms.shoulderRunning) {
            elbowSpeed = (-4/5 * shoulderSpeed) - Constants.kMaxElbowSpeed.get();
            arms.runElbow(elbowSpeed);

          //determine and set elbow speed if shoulder is NOT running
          } else {
            elbowSpeed = -Constants.kMaxElbowSpeed.get();
            arms.runElbow(elbowSpeed);
          }

          if (!controller.getRawButton(XboxController.Button.kA.value)) {
            stop = true;
          }

        //in case something goes wrong
        } else {
          arms.stopElbow();
        }
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arms.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (stop) {
      return true;
    } else {
      return false;
    }
  }
}
