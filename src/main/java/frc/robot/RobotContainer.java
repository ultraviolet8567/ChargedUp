package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Preset;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.odometry.*;
import frc.robot.subsystems.*;
import frc.robot.util.ControllerIO;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    // private final VisionOdometry vision = new VisionOdometry();
    private final GyroOdometry gyro = new GyroOdometry(swerve);
    // private final Odometry odometry = new Odometry(gyro, vision);
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriveControllerPort);
    private static final Joystick armJoystick = new Joystick(OIConstants.kArmControllerPort);

    public final static ShuffleboardTab tabMain = Shuffleboard.getTab("Main");

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            gyro,
            () -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
            () -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
            () -> OIConstants.controllerTypeDrive == ControllerType.JOYSTICK ? driverJoystick.getRawButton(ControllerIO.getTrigger()) : true,
            () -> Constants.fieldOriented));

        arms.setDefaultCommand(new MoveArms(
            arms,
            () -> armJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> armJoystick.getRawAxis(ControllerIO.getRightY()),
            () -> armJoystick.getRawAxis(ControllerIO.getLeftTrigger()),
            () -> armJoystick.getRawAxis(ControllerIO.getRightTrigger())));

        // arms.setDefaultCommand(new ArmManual(
        //     arms,
        //     () -> armJoystick.getRawAxis(ControllerIO.getLeftY()),
        //     () -> armJoystick.getRawAxis(ControllerIO.getRightY())));

        // arms.setDefaultCommand(new MoveToPreset(arms));
        
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(gyro));
        
        // Commands to pickup and drop game pieces
        new JoystickButton(armJoystick, XboxController.Button.kRightBumper.value).whileTrue(new Pickup(intake));
        new JoystickButton(armJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Drop(intake));

        // Commands to change the global game piece value (temporary)
        new JoystickButton(armJoystick, XboxController.Button.kRightStick.value).onTrue(new ChangeGamePiece(GamePiece.CONE));
        new JoystickButton(armJoystick , XboxController.Button.kLeftStick.value).onTrue(new ChangeGamePiece(GamePiece.CUBE));

        // Temporary command to allow for software arm loosening if we lock the chains
        // new JoystickButton(armJoystick , XboxController.Button.kStart.value).onTrue(new ToggleArmIdleMode(arms));
                
        // Change preset target
        // TODO: Controller mapping for these presets
        new JoystickButton(armJoystick, XboxController.Button.kY.value).onTrue(new SetPresetValue(arms, Preset.HIGH_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kX.value).onTrue(new SetPresetValue(arms, Preset.MID_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kB.value).onTrue(new SetPresetValue(arms, Preset.HYBRID_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kBack.value).onTrue(new SetPresetValue(arms, Preset.GROUND_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kStart.value).onTrue(new SetPresetValue(arms, Preset.SUBSTATION_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kA.value).onTrue(new SetPresetValue(arms, Preset.TAXI));
    }

    public Command getAutonomousCommand() {
        return null;
        // return new AutoDriveOut(swerve, odometry);
    }
}