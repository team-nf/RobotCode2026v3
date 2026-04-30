package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.TelemetryConstants;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FuelSim;
import frc.robot.utils.HopperSim;
import frc.robot.utils.MatchTracker;
import frc.robot.utils.ShooterSim;
import frc.robot.utils.SwerveFieldContactSim;

/**
 * Main robot lifecycle class.
 *
 * <p>Keep this class focused on mode transitions and periodic scheduling. Most robot behavior is
 * delegated to {@link RobotContainer} and subsystem/command code.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final MatchTracker matchTracker;

  public Robot() {
    DriverStation.waitForDsConnection(600.0);
    AllianceUtil.refreshAllianceFromDriverStation();
    // Start loggers before RobotContainer so all subsystem log entries are created after
    // the DataLog file is open. Prefers USB stick; falls back to roboRIO storage.
    if (TelemetryConstants.SHOULD_LOG) {
      SignalLogger.start();
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());
    }
    matchTracker = new MatchTracker();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Warm up pathfinding resources once so the first autonomous path request is fast.
    PathfindingCommand.warmupCommand();

    // Use local AD* for on-robot dynamic pathfinding.
    Pathfinding.setPathfinder(new LocalADStar());

    // Cache alliance once on startup; refreshed again on each mode transition.
    AllianceUtil.refreshAllianceFromDriverStation();
    if (TelemetryConstants.SHOULD_LOG) DataLogManager.log("Robot code started");
  }

  @Override
  public void robotPeriodic() {
    // Runs button bindings + scheduled commands each loop.
    CommandScheduler.getInstance().run();

    // Refresh match-phase telemetry/state each loop.
    matchTracker.updateMatchTracker();

    // Container-level periodic tasks (telemetry, machine periodic, sim hooks, etc.).
    m_robotContainer.containerPeriodic();
  }

  @Override
  public void disabledInit() {
    AllianceUtil.refreshAllianceFromDriverStation();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    AllianceUtil.refreshAllianceFromDriverStation();
    // Ask the chooser for the selected auto and schedule it if one is selected.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }

    if (TelemetryConstants.SHOULD_LOG) DataLogManager.log("Autonomous mode started");

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    AllianceUtil.refreshAllianceFromDriverStation();
    // Stop autonomous when teleop starts so driver commands take over cleanly.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (TelemetryConstants.SHOULD_LOG) DataLogManager.log("Teleop mode started");

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Expose SysId dashboard commands only in test mode to avoid periodic dashboard overhead.
    m_robotContainer.publishSysIdCommands();
    // Start test mode from a known clean command state.
    CommandScheduler.getInstance().cancelAll();
    //CommandScheduler.getInstance().schedule(m_robotContainer.getTestCommand());
  }

  @Override
  public void testPeriodic() {}

  private Notifier swerveSimNotrifier;

  @Override
  public void simulationInit() {
    // Separate fixed-rate sim loop for contact/collision handling.
    swerveSimNotrifier =
        new Notifier(
            () -> {
              SwerveFieldContactSim.getInstance().handleSwerveSimFieldCollisions();
            });

    swerveSimNotrifier.startPeriodic(SwerveFieldContactSim.getInstance().simLoopTimeSec);
  }

  @Override
  public void simulationPeriodic() {
    // Advance custom game-piece and mechanism simulations.
    FuelSim.getInstance().updateSim();
    HopperSim.getInstance().updateSim();
    ShooterSim.getInstance().updateSim();
  }
}
