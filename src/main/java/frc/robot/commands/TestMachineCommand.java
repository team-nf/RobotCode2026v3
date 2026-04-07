package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;

/**
 * Manual test command that drives all machine mechanisms from NetworkTables setpoints.
 *
 * <p>Useful for pit validation and subsystem bring-up without redeploying code.
 */
public class TestMachineCommand extends Command {

  // NetworkTables path used by dashboards/pit tools to write test setpoints.
  private static final String TEST_TABLE = "Conf/TestMachine";

  private final TheMachine theMachine;

  // Shooter mechanism test setpoints.
  private final DoubleEntry shooterFlywheelRpsEntry;
  private final DoubleEntry shooterHoodRotEntry;

  // Cargo-path mechanism test setpoints.
  private final DoubleEntry feederBeltRpsEntry;
  private final DoubleEntry feederFeedRpsEntry;
  // Legacy single-feeder entry retained as fallback for older dashboards.
  private final DoubleEntry feederRpsEntry;
  private final DoubleEntry hopperRpsEntry;
  private final DoubleEntry intakeRpsEntry;
  private final DoubleEntry intakeExtensionMetersEntry;

  public TestMachineCommand(TheMachine theMachine) {
    this.theMachine = theMachine;

    // All setpoints live under one table for easy dashboard grouping.
    NetworkTable testTable = NetworkTableInstance.getDefault().getTable(TEST_TABLE);

    // Topic names should match dashboard widgets exactly.
    shooterFlywheelRpsEntry = testTable.getDoubleTopic("ShooterFlywheelRps").getEntry(0.0);
    shooterHoodRotEntry = testTable.getDoubleTopic("ShooterHoodRot").getEntry(0.0);
    feederBeltRpsEntry = testTable.getDoubleTopic("FeederBeltRps").getEntry(0.0);
    feederFeedRpsEntry = testTable.getDoubleTopic("FeederFeedRps").getEntry(0.0);
    feederRpsEntry = testTable.getDoubleTopic("FeederRps").getEntry(0.0);
    hopperRpsEntry = testTable.getDoubleTopic("HopperRps").getEntry(0.0);
    intakeRpsEntry = testTable.getDoubleTopic("IntakeRps").getEntry(0.0);
    intakeExtensionMetersEntry = testTable.getDoubleTopic("IntakeExtensionMeters").getEntry(0.0);

    // Seed defaults so entries appear immediately on dashboards.
    shooterFlywheelRpsEntry.setDefault(0.0);
    shooterHoodRotEntry.setDefault(0.0);
    feederBeltRpsEntry.setDefault(0.0);
    feederFeedRpsEntry.setDefault(0.0);
    feederRpsEntry.setDefault(0.0);
    hopperRpsEntry.setDefault(0.0);
    intakeRpsEntry.setDefault(0.0);
    intakeExtensionMetersEntry.setDefault(0.0);

    // This command owns all machine subsystems while active.
    addRequirements(theMachine.getSubsystems());
  }

  @Override
  public void execute() {
    // Read latest NT values every loop so tuning updates apply immediately.
    double legacyFeederRps = feederRpsEntry.get(0.0);
    theMachine.testWithTurretToHubAndHoodCalculated(
        shooterFlywheelRpsEntry.get(0.0),
        feederBeltRpsEntry.get(legacyFeederRps),
        feederFeedRpsEntry.get(legacyFeederRps),
        hopperRpsEntry.get(0.0),
        intakeRpsEntry.get(0.0),
        intakeExtensionMetersEntry.get(0.0));
  }

  @Override
  public boolean isFinished() {
    // Keep running until button/trigger release (or interruption).
    return false;
  }
}
