package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ControllerType;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Preset;
import frc.robot.commands.ChangeGamePiece;
import frc.robot.commands.Drop;
import frc.robot.commands.MoveArms;
import frc.robot.commands.Pickup;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetPresetValue;
import frc.robot.commands.SwerveTeleOp;
import frc.robot.commands.ToggleSwerveSpeed;
import frc.robot.odometry.GyroOdometry;
import frc.robot.subsystems.Arms;
import frc.robot.subsystems.AutoChooser;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.util.ControllerIO;

public class RobotContainer {
    private static final Swerve swerve = new Swerve();
    private static final Intake intake = new Intake();
    private static final Arms arms = new Arms();
    private static final GyroOdometry gyro = new GyroOdometry(swerve);
    private static final AutoChooser autoChooser = new AutoChooser(swerve, arms, intake, gyro);
    // private final VisionOdometry vision = new VisionOdometry();
    // private final Odometry odometry = new Odometry(gyro, vision);

    private static final Joystick driverJoystick = new Joystick(OIConstants.kDriveControllerPort);
    private static final Joystick armJoystick = new Joystick(OIConstants.kArmControllerPort);

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            gyro,
            () -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getLeftX()),
            () -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
            () -> OIConstants.controllerTypeDrive == ControllerType.JOYSTICK ? driverJoystick.getRawButton(ControllerIO.getTrigger()) : true,
            () -> driverJoystick.getRawButton(ControllerIO.getRightBumper())));

        arms.setDefaultCommand(new MoveArms(
            arms,
            () -> armJoystick.getRawAxis(ControllerIO.getLeftY()),
            () -> armJoystick.getRawAxis(ControllerIO.getRightY())));
        
        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(gyro));

        // Commands to change the max speed to a slower speed for small adjustments
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileTrue(new ToggleSwerveSpeed());

        // Commands to pickup and drop game pieces
        new JoystickButton(armJoystick, XboxController.Button.kRightBumper.value).whileTrue(new Pickup(intake));
        new JoystickButton(armJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Drop(intake));

        // Commands to change the global game piece value
        new JoystickButton(armJoystick, XboxController.Button.kRightStick.value).onTrue(new ChangeGamePiece(GamePiece.CONE));
        new JoystickButton(armJoystick , XboxController.Button.kLeftStick.value).onTrue(new ChangeGamePiece(GamePiece.CUBE));

        // Temporary command to allow for software arm loosening if we lock the chains
        // new JoystickButton(armJoystick , XboxController.Button.kStart.value).onTrue(new ToggleArmIdleMode(arms));
                
        // Change preset target
        new JoystickButton(armJoystick, XboxController.Button.kY.value).onTrue(new SetPresetValue(arms, Preset.HIGH_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kX.value).onTrue(new SetPresetValue(arms, Preset.MID_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kA.value).onTrue(new SetPresetValue(arms, Preset.HYBRID_NODE));
        new JoystickButton(armJoystick, XboxController.Button.kBack.value).onTrue(new SetPresetValue(arms, Preset.GROUND_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kStart.value).onTrue(new SetPresetValue(arms, Preset.SUBSTATION_INTAKE));
        new JoystickButton(armJoystick, XboxController.Button.kB.value).onTrue(new SetPresetValue(arms, Preset.TAXI));

        new POVButton(armJoystick, 0).onTrue(new ChangeGamePiece(true));
        new POVButton(armJoystick, 90).onTrue(new ChangeGamePiece(true));
        new POVButton(armJoystick, 180).onTrue(new ChangeGamePiece(true));
        new POVButton(armJoystick, 270).onTrue(new ChangeGamePiece(true));
    }

    public Command getAutonomousCommand() {
        Logger.getInstance().recordOutput("Auto/Path", autoChooser.getPathName());
        return autoChooser.getAutoCommand();
        // return null;
    }
}