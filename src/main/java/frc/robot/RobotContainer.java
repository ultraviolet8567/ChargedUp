package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoDriveOut;
import frc.robot.odometry.*;
import frc.robot.subsystems.*;
import frc.robot.util.ControllerIO;
import frc.robot.util.ControllerIO.ControllerType;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Arms arms = new Arms();
    private final VisionOdometry vision = new VisionOdometry();
    private final GyroOdometry gyro = new GyroOdometry(swerve);
    private final Odometry odometry = new Odometry(gyro, vision);
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriveControllerPort);
    private static final Joystick armJoystick = new Joystick(OIConstants.kArmControllerPort);

    public final static ShuffleboardTab tabOdometry = Shuffleboard.getTab("Odometry");

    private String gamePiece = "Cone";

    public RobotContainer() {
        swerve.setDefaultCommand(new SwerveTeleOp(
            swerve,
            gyro,
            () -> ControllerIO.inversionY() * driverJoystick.getRawAxis(ControllerIO.getY()),
            () -> ControllerIO.inversionX() * driverJoystick.getRawAxis(ControllerIO.getX()),
            () -> ControllerIO.inversionRot() * driverJoystick.getRawAxis(ControllerIO.getRot()),
            () -> OIConstants.controllerTypeDrive == ControllerType.JOYSTICK ? driverJoystick.getRawButton(ControllerIO.getTrigger()) : true,
            () -> Constants.fieldOriented));

        // arms.setDefaultCommand(new MoveToPreset(arms));
        
        Logger.getInstance().recordOutput("GamePiece", "Cone");

        configureButtonBindings();
    }

    public void configureButtonBindings() {
        new JoystickButton(driverJoystick, XboxController.Button.kBack.value).onTrue(new ResetEncoders(swerve));
        // new JoystickButton(driverJoystick, XboxController.Button.kStart.value).onTrue(new ResetGyro(gyro));
        
        // Commands to pickup and drop game pieces
        new JoystickButton(driverJoystick, XboxController.Button.kRightBumper.value).whileTrue(new Pickup(intake, this));
        new JoystickButton(driverJoystick, XboxController.Button.kLeftBumper.value).whileTrue(new Drop(intake, this));

        // Commands to change the global game piece value (temporary)
        new JoystickButton(driverJoystick, XboxController.Button.kLeftStick.value).onTrue(new ChangeGamePiece(this, "Cone"));
        new JoystickButton(driverJoystick, XboxController.Button.kRightStick.value).onTrue(new ChangeGamePiece(this, "Cube"));
                
        // Manual movement
        new JoystickButton(driverJoystick, XboxController.Button.kX.value).whileTrue(new ManipulateArms(arms, "shoulder", "forward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kY.value).whileTrue(new ManipulateArms(arms, "shoulder", "backward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kB.value).whileTrue(new ManipulateArms(arms, "elbow", "forward"));   
        new JoystickButton(driverJoystick, XboxController.Button.kA.value).whileTrue(new ManipulateArms(arms, "elbow", "backward"));
    
        // Change preset target
        // new JoystickButton(armJoystick, XboxController.Button.kY.value).onTrue(new SetPresetValue(arms, "high node"));
        // new JoystickButton(armJoystick, XboxController.Button.kB.value).onTrue(new SetPresetValue(arms, "mid node"));
        // new JoystickButton(armJoystick, XboxController.Button.kA.value).onTrue(new SetPresetValue(arms, "hybrid node"));
        // new JoystickButton(armJoystick, XboxController.Button.kRightBumper.value).onTrue(new SetPresetValue(arms, "ground intake"));
        // new JoystickButton(armJoystick, XboxController.Button.kLeftBumper.value).onTrue(new SetPresetValue(arms, "high intake"));
        // new JoystickButton(armJoystick, XboxController.Button.kStart.value).onTrue(new SetPresetValue(arms, "starting"));
        // new JoystickButton(armJoystick, XboxController.Button.kBack.value).onTrue(new SetPresetValue(arms, "taxi"));
    }

    public Command getAutonomousCommand() {
        return null;
        // return new AutoDriveOut(swerve, odometry);
    }

    public String getGamePiece() {
        return gamePiece;
    }

    public void setGamePiece(String gamePiece) {
        this.gamePiece = gamePiece;
    } 
}