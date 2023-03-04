package frc.robot;

import javax.naming.ldap.ManageReferralControl;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    // private final Vision vision = new Vision();
    
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private static final Joystick toggleJoystick = new Joystick(OIConstants.kToggleControllerPort);

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

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(swerve));
        //new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).onTrue(new Stop(swerve, intake, arms));

        // these aren't where they are, but rather mere examples for new people who need to see how to configure button bindings
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileTrue(new IntakeCC(intake, "cone"));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new IntakeCC(intake, "cube"));

        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileFalse(new IntakeCC(intake, "cone"));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).whileFalse(new IntakeCC(intake, "cube"));

        new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new ManipulateArms(arms, "shoulder", "forward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new ManipulateArms(arms, "shoulder", "backward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new ManipulateArms(arms, "elbow", "forward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).whileTrue(new ManipulateArms(arms, "elbow", "backward"));
    
        new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileFalse(new StopShoulder(arms));
        new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileFalse(new StopShoulder(arms));   
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileFalse(new StopElbow(arms));   
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).whileFalse(new StopElbow(arms));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}