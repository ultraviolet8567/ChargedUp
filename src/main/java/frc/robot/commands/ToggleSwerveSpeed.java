package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;

public class ToggleSwerveSpeed extends CommandBase {

    public ToggleSwerveSpeed() { }

    @Override
    public void initialize() {
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kTeleDriveSlowSpeedPercentMetersPerSecond;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interuppted) {
        DriveConstants.kTeleDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond * DriveConstants.kTeleDriveMaxSpeedPercentMetersPerSecond;
    }
}