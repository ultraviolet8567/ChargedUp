// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants.ArmConstants;
// import frc.robot.subsystems.Arms;

// public class ManipulateArms extends CommandBase {
//   private Arms arms;
//   private String joint;
//   private String direction;
//   private double elbowSpeed;
//   private double shoulderSpeed;
  
//   public ManipulateArms(Arms arms, String joint, String direction) {
//     this.arms = arms;
//     addRequirements(arms);

//     this.joint = joint;
//     this.direction = direction;

//     elbowSpeed = 0;
//     shoulderSpeed = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     // System.out.println(joint + " " + direction);
//       if (joint == "shoulder") {
        
//         //run shoulder forward
//         if (direction == "forward" && arms.checkShoulderLocationForward()) {
          
//           //determine speeds for shoulder and elbow
//           shoulderSpeed = ArmConstants.kMaxShoulderSpeed.get();
//           elbowSpeed = -4/5 * shoulderSpeed;

//           //set shoulder and elbow to determined speeds
//           arms.runShoulder(shoulderSpeed);
//           arms.runElbow(elbowSpeed);

//         //run shoulder backward
//         } else if (direction == "backward" && arms.checkShoulderLocationBackward()) {
          
//           //determine speeds for shoulder and elbow
//           shoulderSpeed = -ArmConstants.kMaxShoulderSpeed.get();
//           elbowSpeed = -4/5 * shoulderSpeed;

//           //set shoulder and elbow to determined speeds
//           arms.runShoulder(shoulderSpeed);
//           arms.runElbow(elbowSpeed);

//         //in case something goes wrong
//         } else {
//           arms.stopShoulder();
//         }
//       } else if (joint == "elbow") {
        
//         //run elbow forward
//         if (direction == "forward" && arms.checkElbowLocationForward()) {
          
//           //determine and set elbow speed if shoulder is running
//           elbowSpeed = ArmConstants.kMaxElbowSpeed.get();
//           arms.runElbow(elbowSpeed);
          
//           //run elbow backward
//         } else if (direction == "backward" && arms.checkElbowLocationBackward()) {
          
//           //determine and set elbow speed if shoulder is running
//           elbowSpeed = (-4/5 * shoulderSpeed) - ArmConstants.kMaxElbowSpeed.get();
//           arms.runElbow(elbowSpeed);

//         //in case something goes wrong
//         } else {
//           arms.stopElbow();
//         }
//       }
//     }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     arms.stop();
//   }

// }
