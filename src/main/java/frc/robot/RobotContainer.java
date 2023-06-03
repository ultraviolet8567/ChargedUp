package frc.robot;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Preset;
import frc.robot.commands.SetCardinalDirection;
import frc.robot.commands.ChangeGamePiece;
import frc.robot.commands.Drop;
import frc.robot.commands.MoveArms;
import frc.robot.commands.Pickup;
import frc.robot.commands.SetPresetValue;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Lights.GamePiece;
import frc.robot.util.ControllerIO;

public class RobotContainer {
    // Subsystems
    private static final Swerve swerve = new Swerve();
    private static final Intake intake = new Intake();
    private static final Arms arms = new Arms();
    private static final GyroOdometry gyro = new GyroOdometry(swerve);
    private static final AutoChooser autoChooser = new AutoChooser(swerve, arms, intake, gyro);

    // Joysticks
    private static final Joystick driverJoystick = new Joystick(OIConstants.kDriveControllerPort);
    private static final Joystick armJoystick = new Joystick(OIConstants.kArmControllerPort);

    // Cameras
    public final UsbCamera frontCamera = CameraServer.startAutomaticCapture(0);

    public RobotContainer() {
        // Configure default commands for driving and arm movement
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            gyro,
            () -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
            () -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
            () -> OIConstants.controllerTypeDrive == ControllerType.JOYSTICK ? driverJoystick.getRawButton(ControllerIO.getTrigger()) : true,
            () -> driverJoystick.getRawButton(XboxController.Button.kLeftBumper.value),
            () -> driverJoystick.getRawButton(XboxController.Button.kRightBumper.value)));

        arms.setDefaultCommand(new MoveArms(
            arms,
            () -> armJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> armJoystick.getRawAxis(ControllerIO.getRightY())));

        // Send camera to Shuffleboard
        Shuffleboard.getTab("Main").add("Front camera", frontCamera).withWidget(BuiltInWidgets.kCameraStream)
            .withSize(4, 4)
            .withProperties(Map.of("rotation", "HALF"));
        
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        // new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new InstantCommand(() -> swerve.resetEncoders()));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> gyro.resetGyro()));

        // Commands for Cardinal Directions
        // new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new SetCardinalDirection(swerve, 0));
        // new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new SetCardinalDirection(swerve, Math.PI / 2));
        // new JoystickButton(driverJoystick, XboxController.Button.kA.value).whileTrue(new SetCardinalDirection(swerve, Math.PI));
        // new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new SetCardinalDirection(swerve, -Math.PI / 2));

        // Commands to pickup and drop game pieces
        new JoystickButton(armJoystick, XboxController.Button.kRightBumper.value).whileTrue(new Pickup(intake));
        new JoystickButton(armJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Drop(intake));

        // Commands to change the global game piece value
        new JoystickButton(armJoystick, XboxController.Button.kRightStick.value).onTrue(new ChangeGamePiece(GamePiece.CUBE));
        new JoystickButton(armJoystick , XboxController.Button.kLeftStick.value).onTrue(new ChangeGamePiece(GamePiece.CONE));

        // Temporary command to allow for software arm loosening if we lock the chains
        // new JoystickButton(armJoystick , XboxController.Button.kStart.value).onTrue(new ToggleArmIdleMode(arms));
                
        // Change preset target
        new JoystickButton(armJoystick, XboxController.Button.kY.value).onTrue(new SetPresetValue(arms, Preset.HIGH_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kX.value).onTrue(new SetPresetValue(arms, Preset.MID_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kA.value).onTrue(new SetPresetValue(arms, Preset.TAXI));
        new JoystickButton(armJoystick, XboxController.Button.kBack.value).onTrue(new SetPresetValue(arms, Preset.GROUND_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kStart.value).onTrue(new SetPresetValue(arms, Preset.SUBSTATION_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kB.value).onTrue(new SetPresetValue(arms, Preset.TAXI));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getAutoCommand();
    }

    public static Joystick getDriverJoystick() {
        return driverJoystick;
    }

    public Swerve getSwerve() {
        return swerve;
    }
}