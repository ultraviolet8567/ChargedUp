package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.odometry.Gyro;

public class ResetGyro extends CommandBase {
    private Gyro gyro;

    public ResetGyro(Gyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        gyro.resetGyro();
    }
}