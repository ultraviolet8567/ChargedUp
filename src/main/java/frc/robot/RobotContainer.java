package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.odometry.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    private final VisionOdometry vision = new VisionOdometry();
    private final GyroOdometry gyro = new GyroOdometry(swerve);
    private final Odometry odometry = new Odometry(gyro, vision);
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

    public final static ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            gyro,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> Constants.fieldOriented));
            // () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(gyro));
        // new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).onTrue(new Stop(swerve));

        // these aren't where they are, but rather mere examples for new people who need to see how to configure button bindings
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).onTrue(new Cone(intake));
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).onTrue(new Cube(intake));
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileTrue(new TurnElbow(arms));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new TurnShoulder(arms));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}