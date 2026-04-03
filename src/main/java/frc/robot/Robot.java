package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Container;
import frc.robot.utils.FuelSim;
import frc.robot.utils.HopperSim;
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

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Warm up pathfinding resources once so the first autonomous path request is fast.
    PathfindingCommand.warmupCommand();

    // Use local AD* for on-robot dynamic pathfinding.
    Pathfinding.setPathfinder(new LocalADStar());

  }

  @Override
  public void robotPeriodic() {
    // Runs button bindings + scheduled commands each loop.
    CommandScheduler.getInstance().run();

    // Container-level periodic tasks (telemetry, machine periodic, sim hooks, etc.).
    m_robotContainer.containerPeriodic();
  }

  @Override
  public void disabledInit() {
    // Cache alliance color for utilities that need mirrored field logic.
    Container.isBlue = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // Ask the chooser for the selected auto and schedule it if one is selected.
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Stop autonomous when teleop starts so driver commands take over cleanly.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Start test mode from a known clean command state.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private Notifier swerveSimNotrifier;

  @Override
  public void simulationInit() {
    // Separate fixed-rate sim loop for contact/collision handling.
    swerveSimNotrifier = new Notifier(() -> {
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
