// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;
import frc.robot.utils.ShooterCalculator;

/**
 * Manual test command for tuning the pass lookup table.
 *
 * <p>Robot stays stationary while you manually dial in RPS and hood angle via NetworkTables.
 * Turret is auto-aimed at the correct pass target (left or right based on robot Y position).
 * Dashboard shows distance to both pass targets and what the lookup table calculates at that
 * distance so you can compare and update the table row by row.
 */
public class TestPassCommand extends Command {

  private static final String TEST_TABLE = "Conf/TestPass";

  private static final double LANE_SPLIT_Y =
      (PoseConstants.BLUE_PASS_LEFT.getY() + PoseConstants.BLUE_PASS_RIGHT.getY()) * 0.5;

  private final TheMachine theMachine;
  private final CommandSwerveDrivetrain drivetrain;

  private final DoubleEntry passFlywheelRpsEntry;
  private final DoubleEntry passHoodDegEntry;

  public TestPassCommand(TheMachine theMachine, CommandSwerveDrivetrain drivetrain) {
    this.theMachine = theMachine;
    this.drivetrain = drivetrain;

    NetworkTable testTable = NetworkTableInstance.getDefault().getTable(TEST_TABLE);
    passFlywheelRpsEntry = testTable.getDoubleTopic("PassFlywheelRps").getEntry(0.0);
    passHoodDegEntry     = testTable.getDoubleTopic("PassHoodDeg").getEntry(0.0);

    passFlywheelRpsEntry.setDefault(40.0);
    passHoodDegEntry.setDefault(14.0);

    addRequirements(theMachine.getSubsystems());
  }

  // Class-level temps — no allocation in execute().
  private Pose2d robotPose = new Pose2d();
  private Pose2d passLeft  = new Pose2d();
  private Pose2d passRight = new Pose2d();
  private double shooterX;
  private double shooterY;
  private double distToLeft;
  private double distToRight;
  private double distToSelected;
  private boolean onLeftSide;
  private Pose2d selectedTarget = new Pose2d();
  private double[] calcParams;
  private double manualRps;
  private double manualHood;
  private double heading;
  private double robotAngleToPass;
  private double rawAngleError;
  private double turretAngleDeg;

  @Override
  public void execute() {
    robotPose = drivetrain.getPose();
    shooterX  = ShooterCalculator.getShooterXFromRobotPose(robotPose);
    shooterY  = ShooterCalculator.getShooterYFromRobotPose(robotPose);

    passLeft  = Container.isBlue ? PoseConstants.BLUE_PASS_LEFT  : PoseConstants.RED_PASS_LEFT;
    passRight = Container.isBlue ? PoseConstants.BLUE_PASS_RIGHT : PoseConstants.RED_PASS_RIGHT;

    distToLeft  = Math.hypot(passLeft.getX()  - shooterX, passLeft.getY()  - shooterY);
    distToRight = Math.hypot(passRight.getX() - shooterX, passRight.getY() - shooterY);

    onLeftSide     = robotPose.getY() >= LANE_SPLIT_Y;
    selectedTarget = onLeftSide ? passLeft : passRight;
    distToSelected = onLeftSide ? distToLeft : distToRight;

    // Auto-aim turret to the selected pass target (robot is stationary, no speed compensation).
    heading         = robotPose.getRotation().getRadians();
    robotAngleToPass = Math.atan2(
        selectedTarget.getY() - shooterY,
        selectedTarget.getX() - shooterX);
    rawAngleError  = robotAngleToPass - heading;
    turretAngleDeg = Math.toDegrees(Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError)));

    // What the lookup table would command at this distance.
    calcParams = ShooterCalculator.calculatePassParameters(shooterX, shooterY, selectedTarget);

    manualRps  = passFlywheelRpsEntry.get(40.0);
    manualHood = passHoodDegEntry.get(14.0);

    theMachine.passTest(manualRps, manualHood, turretAngleDeg);

    SmartDashboard.putNumber("TestPass/DistToPassLeftM",
        TelemetryConstants.roundTelemetry(distToLeft));
    SmartDashboard.putNumber("TestPass/DistToPassRightM",
        TelemetryConstants.roundTelemetry(distToRight));
    SmartDashboard.putNumber("TestPass/DistToSelectedM",
        TelemetryConstants.roundTelemetry(distToSelected));
    SmartDashboard.putString("TestPass/SelectedTarget", onLeftSide ? "LEFT" : "RIGHT");

    // Lookup table result for this distance — use these to fill in the table row.
    SmartDashboard.putNumber("TestPass/CalcRps",
        TelemetryConstants.roundTelemetry(calcParams[0]));
    SmartDashboard.putNumber("TestPass/CalcHoodDeg",
        TelemetryConstants.roundTelemetry(calcParams[1]));
    SmartDashboard.putNumber("TestPass/CalcFlightTimeSec",
        TelemetryConstants.roundTelemetry(ShooterCalculator.getPassFlightTime(distToSelected)));

    // What you are currently commanding — adjust until the pass lands correctly.
    SmartDashboard.putNumber("TestPass/ManualRps",
        TelemetryConstants.roundTelemetry(manualRps));
    SmartDashboard.putNumber("TestPass/ManualHoodDeg",
        TelemetryConstants.roundTelemetry(manualHood));
    SmartDashboard.putNumber("TestPass/AutoTurretDeg",
        TelemetryConstants.roundTelemetry(turretAngleDeg));

    SmartDashboard.putBoolean("TestPass/IsPassReady", theMachine.isPassReady());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
