package frc.robot;

import java.util.Map;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Lights.GamePiece;
import frc.robot.subsystems.Lights.RobotState;
import frc.robot.util.VirtualSubsystem;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private static GamePiece gamePiece;
    private static GenericEntry gamePieceBox, postTime;
    private static SendableChooser<GamePiece> initialGamePiece;
    private static Logger logger;

    @Override
    public void robotInit() {
        logger = Logger.getInstance();
        Lights.getInstance();

        System.out.println("[Init] Starting AdvantageKit");

        switch (Constants.currentMode) {
            case TUNING:
            case REAL:
                logger.addDataReceiver(new WPILOGWriter(Constants.logpath));
                logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                String logpath = LogFileUtil.findReplayLog();
                logger.setReplaySource(new WPILOGReader(logpath));
                logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logpath, "_sim")));
                break;
        }
        
        logger.start();

        // Instantiate our RobotContainer
        m_robotContainer = new RobotContainer();

        initialGamePiece = new SendableChooser<>();
        initialGamePiece.setDefaultOption("Cone", GamePiece.CONE);
        initialGamePiece.addOption("Cube", GamePiece.CUBE);
        gamePiece = initialGamePiece.getSelected();

        Shuffleboard.getTab("Main").add("Initial game piece", initialGamePiece).withWidget(BuiltInWidgets.kComboBoxChooser)
            .withSize(2, 1)
            .withPosition(4, 1);

        gamePieceBox = Shuffleboard.getTab("Main").add("Game piece", true).withWidget(BuiltInWidgets.kBooleanBox)
            .withProperties(Map.of("color when true", "Yellow", "color when false", "Purple"))
            .withSize(4, 4)
            .withPosition(6, 0)
            .getEntry();

        postTime = Shuffleboard.getTab("Main").add("Time left", 0).withWidget(BuiltInWidgets.kNumberBar)
            .withProperties(Map.of("min", 0, "max", 135))
            .withPosition(4, 0)
            .withSize(2, 1)
            .getEntry();

        logger.recordOutput("GamePiece", gamePiece.toString());
    }

    @Override
    public void robotPeriodic() {
        VirtualSubsystem.periodicAll();
        CommandScheduler.getInstance().run();

        // Update Shuffleboard
        gamePieceBox.setBoolean(Lights.getInstance().gamePiece == GamePiece.CONE);
        postTime.setDouble(DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {
        Lights.getInstance().state = RobotState.DISABLED;
    }

    @Override
    public void disabledPeriodic() {
        gamePiece = initialGamePiece.getSelected();
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }

        Lights.getInstance().gamePiece = initialGamePiece.getSelected();

        // Set state to auto
        Lights.getInstance().state = RobotState.AUTO;

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // set state to teleop
        Lights.getInstance().state = RobotState.TELEOP;

    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}