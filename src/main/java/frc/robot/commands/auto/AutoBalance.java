package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Preset;
import frc.robot.odometry.GyroOdometry;
import frc.robot.Robot;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    private Swerve swerve;
    private GyroOdometry gyro;
    private Timer timer;
    private PIDController pid;

    public AutoBalance(Swerve swerve, GyroOdometry gyro) {
        this.swerve = swerve;
        this.gyro = gyro;
        timer = new Timer();

        pid = new PIDController(0.5, 0, 0);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        Logger.getInstance().recordOutput("Auto/Timer", timer.get());

        double value = pid.calculate(gyro.getRotation3d().getY(), 0);
        Logger.getInstance().recordOutput("Auto/Value", value);

        // if (timeEquals(5)) {
        //     intake.drop(Robot.getGamePiece());
        // }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();

        // arms.setPresetValue(Preset.TAXI);
        // intake.stop();
    }

    public boolean isFinished() {
        return timeEquals(7);
    }

    public boolean timeEquals(double target) {
        return Math.abs(timer.get() - target) < 0.1;
    }
}
