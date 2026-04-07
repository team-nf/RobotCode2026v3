package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;

/**
 * Manual test command that drives all machine mechanisms from NetworkTables setpoints.
 *
 * <p>Useful for pit validation and subsystem bring-up without redeploying code.
 */
public class TestShootCommand extends Command {

  // NetworkTables path used by dashboards/pit tools to write test setpoints.
  private static final String TEST_TABLE = "Conf/TestMachine";

  private final TheMachine theMachine;

  // Shooter mechanism test setpoints.
  private final DoubleEntry shooterFlywheelRpsEntry;
  private final DoubleEntry shooterHoodRotEntry;

  public TestShootCommand(TheMachine theMachine) {
    this.theMachine = theMachine;

    // All setpoints live under one table for easy dashboard grouping.
    NetworkTable testTable = NetworkTableInstance.getDefault().getTable(TEST_TABLE);

    // Topic names should match dashboard widgets exactly.
    shooterFlywheelRpsEntry = testTable.getDoubleTopic("ShooterFlywheelRps").getEntry(0.0);
    shooterHoodRotEntry = testTable.getDoubleTopic("ShooterHoodRot").getEntry(0.0);

    // Seed defaults so entries appear immediately on dashboards.
    shooterFlywheelRpsEntry.setDefault(0.0);
    shooterHoodRotEntry.setDefault(0.0);

    // This command owns all machine subsystems while active.
    addRequirements(theMachine.getSubsystems());
  }

  @Override
  public void execute() {
    // Read latest NT values every loop so tuning updates apply immediately.
    theMachine.shootTest(
        shooterFlywheelRpsEntry.get(0.0),
        shooterHoodRotEntry.get(0.0),
        theMachine.getTurretAngleToHub());

    double[] shooterValuesForHub = theMachine.getShooterValuesForHub();

    SmartDashboard.putNumber("Test/ShooterCalculatedHoodAngle", shooterValuesForHub[1]);
    SmartDashboard.putNumber("Test/ShooterCalculatedRPS", shooterValuesForHub[0]);
  }

  @Override
  public boolean isFinished() {
    // Keep running until button/trigger release (or interruption).
    return false;
  }
}
