package frc.robot;

import java.util.Map;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.GamePiece;
import frc.robot.subsystems.Swerve;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    private static GamePiece gamePiece;
    private static GenericEntry gamePieceBox, postTime;
    private static SendableChooser<GamePiece> initialGamePiece;
    Logger logger;
    long startTime;

    @Override
    public void robotInit() {
        logger = Logger.getInstance();

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

        Logger.getInstance().recordOutput("GamePiece", toString(gamePiece));
    }

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        gamePieceBox.setBoolean(getGamePiece() == GamePiece.CONE);
        postTime.setDouble(135 - Logger.getInstance().getTimestamp() / 1000000.0);
    }

    @Override
    public void disabledInit() {}

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

        gamePiece = initialGamePiece.getSelected();
        startTime = logger.getTimestamp();
    }

    @Override
    public void autonomousPeriodic() {
        if (logger.getTimestamp() - startTime >= 15000000.0)
            m_robotContainer.getSwerve().lockWheels();
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        if (logger.getTimestamp() - startTime >= 153000000.0)
            m_robotContainer.getSwerve().lockWheels();
    }

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

    // Global gamepiece switch
    public static GamePiece getGamePiece() {
        return gamePiece;
    }

    public static void setGamePiece(GamePiece gp) {
        gamePiece = gp;
    }

    public static String toString(GamePiece gp) {
        return gp == GamePiece.CONE ? "Cone" : "Cube";
    }
}