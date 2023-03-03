package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arms;

public class TurnElbow extends CommandBase {
  private Arms arms;
  
  public TurnElbow(Arms subsystem) {
    arms = subsystem;
    addRequirements(arms);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arms.turnElbow();
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
