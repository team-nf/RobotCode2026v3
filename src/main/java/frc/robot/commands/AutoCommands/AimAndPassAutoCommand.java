// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.TheMachine;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;
import frc.robot.utils.ShooterCalculator;

/**
 * Autonomous variant of pass aiming command.
 *
 * <p>Selects the nearest pass lane target and drives machine pass setpoints without writing manual
 * drive controls.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndPassAutoCommand extends Command {

  private final DoubleEntry passAngleErrorEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/PASS/AimAngleError").getEntry(0.0);

  private final DoubleEntry passAimPoseXEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/PASS/AimPoseX").getEntry(0.0);
  private final DoubleEntry passAimPoseYEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/PASS/AimPoseY").getEntry(0.0);
  private final DoubleEntry passAimPoseZEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/PASS/AimPoseZ").getEntry(0.0);

  private final BooleanEntry passOnTargetEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("/PASS/AimOnTarget").getEntry(false);

  private CommandSwerveDrivetrain swerveDrivetrain;
  private TheMachine theMachine;

  private Pose2d passAimPose;

  private static final int FILTER_SIZE = 2;
  private final double[] angleErrorBuffer = new double[FILTER_SIZE];
  private int bufferIndex = 0;

  private static final double TURRET_TOLERANCE_DEG =
      ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);

  /** Creates a new AimAndPassAutoCommand. */
  public AimAndPassAutoCommand(
      CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.theMachine = theMachine;

    addRequirements(theMachine.getSubsystems());

    passAimPose = Container.isBlue ? PoseConstants.BLUE_PASS_RIGHT : PoseConstants.RED_PASS_RIGHT;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < FILTER_SIZE; i++) {
      angleErrorBuffer[i] = 0.0;
    }
    bufferIndex = 0;

    passAimPose = Container.isBlue ? PoseConstants.BLUE_PASS_RIGHT : PoseConstants.RED_PASS_RIGHT;
  }

  private Pose2d robotPose = new Pose2d();
  private double filteredAngleError = 0.0;

  private double velocityRPS = 0.0;
  private double hoodAngle = 0.0;
  private double turretAngleDeg = 0.0;

  double laneSplitY;
  boolean onLeftSide;

  private Pose2d selectPassAimPose(Pose2d currentPose) {
    // Split field into left/right pass lanes using midpoint between lane targets.
    laneSplitY = (PoseConstants.BLUE_PASS_LEFT.getY() + PoseConstants.BLUE_PASS_RIGHT.getY()) * 0.5;
    onLeftSide = currentPose.getY() >= laneSplitY;

    if (Container.isBlue) {
      return onLeftSide ? PoseConstants.BLUE_PASS_LEFT : PoseConstants.BLUE_PASS_RIGHT;
    }
    return onLeftSide ? PoseConstants.RED_PASS_LEFT : PoseConstants.RED_PASS_RIGHT;
  }

  double aimX;
  double aimY;

  double heading;
  double shooterX;
  double shooterY;
  double robotAngleToPass;
  double rawAngleError;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1) Determine current pass target from robot lane side.
    robotPose = swerveDrivetrain.getPose();
    heading = robotPose.getRotation().getRadians();
    shooterX = ShooterCalculator.getShooterXFromRobotState(robotPose.getX(), heading);
    shooterY = ShooterCalculator.getShooterYFromRobotState(robotPose.getY(), heading);
    passAimPose = selectPassAimPose(robotPose);

    filteredAngleError = 0.0;
    for (int i = 0; i < FILTER_SIZE; i++) {
      filteredAngleError += angleErrorBuffer[i];
    }
    filteredAngleError /= FILTER_SIZE;

    aimX = passAimPose.getX();
    aimY = passAimPose.getY();

    robotAngleToPass = Math.atan2(aimY - shooterY, aimX - shooterX);
    rawAngleError = robotAngleToPass - heading;
    rawAngleError = Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError));

    angleErrorBuffer[bufferIndex] = rawAngleError;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    turretAngleDeg =
        Math.toDegrees(
            Math.atan2(Math.sin(robotAngleToPass - heading), Math.cos(robotAngleToPass - heading)));

    // 2) Solve pass setpoints and gate feed on shooter readiness.
    velocityRPS = ShooterCalculator.calculatePassSpeedFromCurrentState(robotPose.getX(), heading);
    hoodAngle = ShooterCalculator.calculatePassHoodAngle();

    if (theMachine.isPassReady()) {
      theMachine.pass(velocityRPS, hoodAngle, turretAngleDeg);
    } else {
      theMachine.getReadyPass(velocityRPS, hoodAngle, turretAngleDeg);
    }

    passOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
    passAngleErrorEntry.set(Math.toDegrees(filteredAngleError));
    passAimPoseXEntry.set(aimX);
    passAimPoseYEntry.set(aimY);
    passAimPoseZEntry.set(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
