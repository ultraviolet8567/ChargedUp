package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

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
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).onTrue(new Pickup(intake));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).onTrue(new Pickup(intake));

        // new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileFalse(new Pickup(intake));
        // new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).whileFalse(new Pickup(intake));
        
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

    public static String getGamePiece() {
        /* Output should come based on the reading from the switch
         * For now we're just basing it off of the buttons from the Xbox
         * Since the right bumper runs the intake for a cone, I've let that return "cone"
         * Since the left bumper runs the intake for a cube, I've let that return "cube"
         */
        Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

        if (driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)) {
            return "Cone";
        }
        else if (driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value)) {
            return "Cube";
        }
        else {
            return "";
        }
    }
}