// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.TheMachine;
import frc.robot.constants.Dimensions;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShooterCalculator;

/**
 * Autonomous variant of hub aiming + shooting assist.
 *
 * <p>Unlike teleop version, this command does not write drive requests and is intended to run
 * alongside an auto path/positioning command.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootAutoCommand extends Command {

  private final DoubleEntry aimAngleErrorEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/AIM/AimAngleError").getEntry(0.0);

  private final DoubleEntry aimPoseXEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/AIM/AimPoseX").getEntry(0.0);
  private final DoubleEntry aimPoseYEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/AIM/AimPoseY").getEntry(0.0);
  private final DoubleEntry aimPoseZEntry =
      NetworkTableInstance.getDefault()
          .getDoubleTopic("/AIM/AimPoseZ")
          .getEntry(Dimensions.HUB_HEIGHT.in(Meters));
  private final BooleanEntry aimOnTargetEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("/AIM/AimOnTarget").getEntry(false);

  private CommandSwerveDrivetrain swerveDrivetrain;
  private TheMachine theMachine;

  private Pose2d hubAimPose;

  // Moving average buffers for noise reduction
  private static final int FILTER_SIZE = 3; // 3-sample moving average
  private final double[] speedXBuffer = new double[FILTER_SIZE];
  private final double[] speedYBuffer = new double[FILTER_SIZE];
  private final double[] angleErrorBuffer = new double[FILTER_SIZE];
  private int bufferIndex = 0;
  private int validSampleCount = 0;

  /** Creates a new AimAndShootAutoCommand. */
  public AimAndShootAutoCommand(
      CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.theMachine = theMachine;

    addRequirements(theMachine.getSubsystems());

    hubAimPose = AllianceUtil.getHubAimPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear moving average buffers
    for (tempLoopIndex = 0; tempLoopIndex < FILTER_SIZE; tempLoopIndex++) {
      speedXBuffer[tempLoopIndex] = 0.0;
      speedYBuffer[tempLoopIndex] = 0.0;
      angleErrorBuffer[tempLoopIndex] = 0.0;
    }
    bufferIndex = 0;
    validSampleCount = 0;

    // Re-resolve alliance hub target in case DS alliance changed while disabled.
    hubAimPose = AllianceUtil.getHubAimPose();
  }

  private Pose2d robotPose = new Pose2d();
  double robotAngleToHub = 0;

  double heading;
  ChassisSpeeds speeds;

  double distance;
  double time;
  double aimX;
  double aimY;

  double[] shootParams;
  double velocityRPS;
  double hoodAngle;
  double turretAngleDeg;

  double filteredSpeedX = 0.0, filteredSpeedY = 0.0, filteredAngleError = 0.0;
  private static final double TURRET_TOLERANCE_DEG =
      ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);
  private static final double TURRET_LOOKAHEAD_SEC = ShooterConstants.TURRET_LOOKAHEAD_SEC;

  double rawAngleError = 0.0;
  double shooterSpeedX = 0.0;
  double shooterSpeedY = 0.0;
  double predictedHeading = 0.0;
  double predictedShooterX = 0.0;
  double predictedShooterY = 0.0;
  double shooterPoseX = 0.0;
  double shooterPoseY = 0.0;
  double shooterOffsetFieldX = 0.0;
  double shooterOffsetFieldY = 0.0;
  double headingSin = 0.0;
  double headingCos = 0.0;
  double sumSin = 0.0;
  double sumCos = 0.0;
  int tempLoopIndex = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1) Gather drivetrain state and derive shooter point kinematics.
    robotPose = swerveDrivetrain.getPose();
    speeds = swerveDrivetrain.getFieldSpeeds();
    heading = robotPose.getRotation().getRadians();
    headingSin = Math.sin(heading);
    headingCos = Math.cos(heading);

    shooterOffsetFieldX =
        TheMachineConstants.SHOOTER_ROTATION_AXIS.getX() * headingCos
            - TheMachineConstants.SHOOTER_ROTATION_AXIS.getY() * headingSin;
    shooterOffsetFieldY =
        TheMachineConstants.SHOOTER_ROTATION_AXIS.getX() * headingSin
            + TheMachineConstants.SHOOTER_ROTATION_AXIS.getY() * headingCos;

    shooterPoseX = robotPose.getX() + shooterOffsetFieldX;
    shooterPoseY = robotPose.getY() + shooterOffsetFieldY;

    // Convert robot-relative CoR offset to field frame and compute shooter-point velocity:
    // v_point = v_center + omega x r
    shooterSpeedX = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * shooterOffsetFieldY;
    shooterSpeedY = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * shooterOffsetFieldX;

    distance = Math.hypot(hubAimPose.getX() - shooterPoseX, hubAimPose.getY() - shooterPoseY);
    time = ShooterCalculator.flightTimeOfFuelFormula(distance);

    filteredSpeedX = 0.0;
    filteredSpeedY = 0.0;
    filteredAngleError = 0.0;

    // Update speed buffers before filtering so newest sample contributes this cycle.
    speedXBuffer[bufferIndex] = shooterSpeedX;
    speedYBuffer[bufferIndex] = shooterSpeedY;
    if (validSampleCount < FILTER_SIZE) {
      validSampleCount++;
    }

    // Calculate filtered (smoothed) values
    for (tempLoopIndex = 0; tempLoopIndex < validSampleCount; tempLoopIndex++) {
      filteredSpeedX += speedXBuffer[tempLoopIndex];
      filteredSpeedY += speedYBuffer[tempLoopIndex];
    }

    filteredSpeedX /= validSampleCount;
    filteredSpeedY /= validSampleCount;

    // 2) Predict where shooter/chassis will be when turret reaches setpoint.
    predictedShooterX = shooterPoseX + filteredSpeedX * TURRET_LOOKAHEAD_SEC;
    predictedShooterY = shooterPoseY + filteredSpeedY * TURRET_LOOKAHEAD_SEC;
    predictedHeading = heading + speeds.omegaRadiansPerSecond * TURRET_LOOKAHEAD_SEC;

    aimX = hubAimPose.getX() - (filteredSpeedX * time);
    aimY = hubAimPose.getY() - (filteredSpeedY * time);

    robotAngleToHub = Math.atan2(aimY - predictedShooterY, aimX - predictedShooterX);
    rawAngleError = robotAngleToHub - predictedHeading;
    // Normalize angle error to [-π, π]
    rawAngleError = Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError));

    // Update angle buffer and compute wrap-safe filtered angle.
    angleErrorBuffer[bufferIndex] = rawAngleError;

    sumSin = 0.0;
    sumCos = 0.0;
    for (tempLoopIndex = 0; tempLoopIndex < validSampleCount; tempLoopIndex++) {
      sumSin += Math.sin(angleErrorBuffer[tempLoopIndex]);
      sumCos += Math.cos(angleErrorBuffer[tempLoopIndex]);
    }
    filteredAngleError = Math.atan2(sumSin, sumCos);

    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    // 3) Solve shooter parameters from filtered motion estimate and predicted aim point.
    shootParams =
        ShooterCalculator.calculateShootingParameters(
            filteredSpeedX, filteredSpeedY, shooterPoseX, shooterPoseY, time);
    velocityRPS = shootParams[0];
    hoodAngle = shootParams[1];
    turretAngleDeg =
        Math.toDegrees(
            Math.atan2(
                Math.sin(robotAngleToHub - predictedHeading),
                Math.cos(robotAngleToHub - predictedHeading)));

    // Feed only once shooter is ready; otherwise stay in spin-up state.
    if (theMachine.isShooterReady()) {
      theMachine.shoot(velocityRPS, hoodAngle, turretAngleDeg);
    } else {
      theMachine.getReady(velocityRPS, hoodAngle, turretAngleDeg);
    }

    aimOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
    aimAngleErrorEntry.set(Math.toDegrees(filteredAngleError));

    if (Robot.isSimulation()) {
      // Publish simulated aiming telemetry for dashboards/3D overlays.
      aimPoseXEntry.set(aimX);
      aimPoseYEntry.set(aimY);
      aimPoseZEntry.set(Dimensions.HUB_HEIGHT.in(Meters));
    }
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
