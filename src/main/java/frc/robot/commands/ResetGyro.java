package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.odometry.GyroOdometry;

public class ResetGyro extends CommandBase {
    private GyroOdometry gyro;

    public ResetGyro(GyroOdometry gyro) {
        this.gyro = gyro;
    }

    @Override
    public void initialize() {
        gyro.resetGyro();
    }
}