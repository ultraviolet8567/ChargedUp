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
import frc.robot.util.ControllerIO;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    // private final Vision vision = new Vision();
    
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    // private static final Joystick armJoystick = new Joystick(OIConstants.kToggleControllerPort);

    public final static ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

    public static String gamePiece = "";

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            () -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getY()),
            () -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getX()),
            () -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
            () -> driverJoystick.getRawButton(ControllerIO.getTrigger()),
            () -> Constants.fieldOriented));
            // () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(swerve));
        
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).onTrue(new Pickup(intake, "Cone"));
        // new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).onTrue(new Pickup(intake, "Cube"));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).onTrue(new StopIntake(intake));
        
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

        // new JoystickButton(armJoystick, XboxController.Button.kA.value).onTrue(new PresetArms(arms, "high node"));
        // new JoystickButton(armJoystick, XboxController.Button.kB.value).onTrue(new PresetArms(arms, "mid node"));
        // new JoystickButton(armJoystick, XboxController.Button.kX.value).onTrue(new PresetArms(arms, "hybrid node"));
        // new JoystickButton(armJoystick, XboxController.Button.kY.value).onTrue(new PresetArms(arms, "ground intake"));
        // new JoystickButton(armJoystick, XboxController.Button.kLeftBumper.value).onTrue(new PresetArms(arms, "high intake"));
        // new JoystickButton(armJoystick, XboxController.Button.kRightBumper.value).onTrue(new PresetArms(arms, "starting"));
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