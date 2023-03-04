package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    private final Vision vision = new Vision();
    
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private static XboxController xbox = new XboxController(0);

    public final static ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> Constants.fieldOriented));
            // () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));

        configureButtonBindings();
    }

    public static String gamePiece() {
        if (xbox.getAButton()) {
            return "cone";
        } else if (!xbox.getBButton()) {
            return "cube";
        }
        return "";
    }

    public static String direction() {
        if (xbox.getRightBumperPressed()) {
            return "forward";
        } else if (xbox.getLeftBumperPressed()) {
            return "backward";
        }
        return "";
    }

    public static String joint() {
        if (xbox.getXButton()) {
            return "shoulder";
        } else if (xbox.getYButton()) {
            return "elbow";
        }
        return "";
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(swerve));
        // new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).onTrue(new Stop(swerve));

        // these aren't where they are, but rather mere examples for new people who need to see how to configure button bindings
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).onTrue(new IntakeCC(intake));
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).onTrue(new IntakeCC(intake));

        new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new ManipulateArms(arms));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}