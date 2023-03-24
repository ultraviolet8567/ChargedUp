package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Arms;

public class ArmManual extends CommandBase {
    private Arms arms;
    private Supplier<Double> leftJoystickSupplier, rightJoystickSupplier;
    private SlewRateLimiter shoulderLimiter, elbowLimiter;

    public ArmManual(Arms arms, Supplier<Double> leftJoystickSupplier, Supplier<Double> rightJoystickSupplier) {
        this.arms = arms;
        this.leftJoystickSupplier = leftJoystickSupplier;
        this.rightJoystickSupplier = rightJoystickSupplier;

        shoulderLimiter = new SlewRateLimiter(ArmConstants.kMaxShoulderAcceleration.get());
        elbowLimiter = new SlewRateLimiter(ArmConstants.kMaxElbowAcceleration.get());

        addRequirements(arms);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get real-time joystick inputs
        double leftJoystick = leftJoystickSupplier.get();
        double rightJoystick = rightJoystickSupplier.get();

        // Apply deadband (drops to 0 if joystick value is less than the deadband)
        leftJoystick = Math.abs(leftJoystick) > OIConstants.kDeadband ? leftJoystick : 0;
        rightJoystick = Math.abs(rightJoystick) > OIConstants.kDeadband ? rightJoystick : 0;

        // Make the driving smoother by using a slew rate limiter to minimize acceleration
        // And scale joystick input to m/s
        double shoulderSpeed = shoulderLimiter.calculate(leftJoystick) * ArmConstants.kMaxShoulderSpeed.get();
        double elbowSpeed = elbowLimiter.calculate(rightJoystick) * ArmConstants.kMaxElbowSpeed.get();

        if (arms.shoulderMovable(shoulderSpeed)) {
            arms.runShoulder(shoulderSpeed);

            // Turn the speed a bit more to account for the shoulder rotation
            elbowSpeed += ArmConstants.kArmsToElbow * shoulderSpeed;
        }
        else {
            arms.stopShoulder();
        }
        
        if (arms.elbowMovable(elbowSpeed)) {
            arms.runElbow(elbowSpeed);
        }
        else {
            arms.stopElbow();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}