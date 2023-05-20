package frc.robot.commands.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Swerve;

// Improving the charge station balance to be more consistent and faster is on that list right?

public class AutoBalance extends CommandBase {
    private final Swerve swerve;
    private final GyroOdometry gyro;
    private Timer timer;
    private PIDController pid;

    public AutoBalance(Swerve swerve, GyroOdometry gyro) {
        this.swerve = swerve;
        this.gyro = gyro;
        addRequirements(swerve);

        timer = new Timer();

        if (DriverStation.getAlliance() == Alliance.Red) {
            pid = new PIDController(2.75, 0, 0);
        } else {
            pid = new PIDController(2.25, 0, 0);
        }
        pid.setTolerance(0.1);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double speed;
        double robotX = gyro.getRotation3d().getX();

        if (Math.abs(robotX) != 0) {
            speed = pid.calculate(robotX, 0);
        } else if (Math.abs(robotX) == 0 && timer.get() <= 5) {
            speed = 0.85;
        } else {
            speed = 0.0;
        }

        // increase or decrease the angle (default 5) if this doesn't work as intended
        if (Math.abs(0 - gyro.getRotation3d().getX()) <= 5) {
            falter(speed);
        } else {
            run(speed);
        }
        
        Logger.getInstance().recordOutput("Auto/Timer", timer.get());
        Logger.getInstance().recordOutput("Auto/Speed", speed);
        Logger.getInstance().recordOutput("Auto/PIDError", pid.getPositionError());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        // Lock the wheels at the end of auto to prevent slipping off the charge station
        swerve.lockWheels();
    }

    public boolean isFinished() {
        return pid.atSetpoint() || (DriverStation.getMatchTime() >= 0.0 && DriverStation.getMatchTime() < Constants.matchEndThreshold);
    }

    // change wait value after testing
    public void falter(double speed) {
        run(0.0);
        wait(1);
        run(speed);
    }

    // waits given time
    public void wait(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            System.out.println("there was an interrupted exception! what the");
        }
    }

    // runs given speed
    public void run(double speed) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speed, 0, 0, gyro.getRotation2d());
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerve.setModuleStates(moduleStates);
    }
        
}
