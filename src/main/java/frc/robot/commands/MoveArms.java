package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Preset;
import frc.robot.subsystems.Arms;

public class MoveArms extends CommandBase {
    private Arms arms;
    private Supplier<Double> leftJoystickSupplier, rightJoystickSupplier;
    private SlewRateLimiter shoulderLimiter, elbowLimiter;

    public MoveArms(Arms arms, Supplier<Double> leftJoystickSupplier, Supplier<Double> rightJoystickSupplier) {
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
        if (arms.idle()) {
            arms.stop();
        }
        else {
            if (arms.getPresetValue() == Preset.MANUAL_OVERRIDE || Math.abs(leftJoystickSupplier.get()) > 1.5 * OIConstants.kDeadband || Math.abs(rightJoystickSupplier.get()) > 1.5 * OIConstants.kDeadband) {
                // Manual arm movement
                arms.setPresetValue(Preset.MANUAL_OVERRIDE);

                // Get real-time joystick inputs
                double leftJoystick = leftJoystickSupplier.get();
                double rightJoystick = rightJoystickSupplier.get();

                // Apply deadband (drops to 0 if joystick value is less than the deadband)
                leftJoystick = Math.abs(leftJoystick) > OIConstants.kDeadband ? leftJoystick : 0;
                rightJoystick = Math.abs(rightJoystick) > OIConstants.kDeadband ? rightJoystick : 0;

                // Make the driving smoother by using a slew rate limiter to minimize acceleration
                // And scale joystick input to m/s
                double shoulderSpeed = shoulderLimiter.calculate(leftJoystick) * ArmConstants.kMaxShoulderSpeedPercentage;
                double elbowSpeed = elbowLimiter.calculate(rightJoystick) * ArmConstants.kMaxElbowSpeedPercentage;

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
            else {
                // Preset automatic movement
                double[] setpoints = arms.getPreset();
                double[] speeds = arms.calculateMotorSpeeds(setpoints[0], setpoints[1]);
                
                double shoulderSpeed = speeds[0];
                double elbowSpeed = speeds[1];

                if (arms.shoulderMovable(shoulderSpeed))
                    arms.runShoulder(shoulderSpeed);
                else
                    arms.stopShoulder();

                if (arms.elbowMovable(elbowSpeed) && arms.elbowPresetMovable(shoulderSpeed))
                    arms.runElbow(elbowSpeed);
                else
                    arms.stopElbow();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arms.stop();
    }
}