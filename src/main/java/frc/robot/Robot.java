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

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand();
    Pathfinding.setPathfinder(new LocalADStar());

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.containerPeriodic();
  }

  @Override
  public void disabledInit() {
    Container.isBlue = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  private Notifier swerveSimNotrifier;

  @Override
  public void simulationInit() {
    
    swerveSimNotrifier = new Notifier(() -> {
      SwerveFieldContactSim.getInstance().handleSwerveSimFieldCollisions();
    });
    
    swerveSimNotrifier.startPeriodic(SwerveFieldContactSim.getInstance().simLoopTimeSec);
    
  }

  @Override
  public void simulationPeriodic() {
    
    FuelSim.getInstance().updateSim();
    HopperSim.getInstance().updateSim();
    ShooterSim.getInstance().updateSim();
    
  }
}
