// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.TheMachine;
import frc.robot.constants.Dimensions;
import frc.robot.constants.MoveNShootConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShooterCalculator;

/**
 * Autonomous variant of shoot aiming command.
 *
 * <p>Solves flywheel/hood/turret setpoints toward the alliance hub without writing manual drive
 * controls — the auto routine owns the drivetrain.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootAutoCommand extends Command {

  private final DoubleEntry aimAngleErrorEntry =
      NetworkTableInstance.getDefault().getDoubleTopic("/AIM/AimAngleError").getEntry(0.0);

  private final StructEntry<Pose3d> aimPose3d =
      NetworkTableInstance.getDefault()
          .getStructTopic("/AIM/AimPose3d", Pose3d.struct)
          .getEntry(new Pose3d());

  private final BooleanEntry aimOnTargetEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("/AIM/AimOnTarget").getEntry(false);

  private final BooleanEntry telemetryEnabledEntry =
      NetworkTableInstance.getDefault().getBooleanTopic("Conf/EnableTelemetry").getEntry(false);

  private final CommandSwerveDrivetrain swerveDrivetrain;
  private final TheMachine theMachine;

  private Pose2d hubAimPose;

  private final double[] speedXBuffer = new double[MoveNShootConstants.SHOOT_FILTER_SIZE];
  private final double[] speedYBuffer = new double[MoveNShootConstants.SHOOT_FILTER_SIZE];
  private final double[] angleErrorBuffer = new double[MoveNShootConstants.SHOOT_FILTER_SIZE];
  private int bufferIndex = 0;
  private int validSampleCount = 0;

  private static final double TURRET_TOLERANCE_DEG =
      ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);
  private static final double TURRET_LOOKAHEAD_SEC = MoveNShootConstants.TURRET_LOOKAHEAD_SEC;
  private static final double SHOOTER_SETPOINT_RPS_DEADBAND = MoveNShootConstants.SETPOINT_RPS_DEADBAND;
  private static final double SHOOTER_SETPOINT_HOOD_DEG_DEADBAND = MoveNShootConstants.SETPOINT_HOOD_DEG_DEADBAND;
  private static final double SHOOTER_SETPOINT_TURRET_DEG_DEADBAND = MoveNShootConstants.SETPOINT_TURRET_DEG_DEADBAND;

  private final Timer executionTimer = new Timer();
  private boolean executionTimeoutElapsed = false;

  private boolean hasSentShooterSetpoint = false;
  private double lastSentVelocityRps = 0.0;
  private double lastSentHoodAngleDeg = 0.0;
  private double lastSentTurretAngleDeg = 0.0;
  private boolean lastSentShootMode = false;

  /** Creates a new AimAndShootAutoCommand. */
  public AimAndShootAutoCommand(
      CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    this.swerveDrivetrain = drivetrain;
    this.theMachine = theMachine;

    addRequirements(theMachine.getSubsystems());

    hubAimPose = AllianceUtil.getHubAimPose();
  }

  @Override
  public void initialize() {
    for (int i = 0; i < MoveNShootConstants.SHOOT_FILTER_SIZE; i++) {
      speedXBuffer[i] = 0.0;
      speedYBuffer[i] = 0.0;
      angleErrorBuffer[i] = 0.0;
    }
    bufferIndex = 0;
    validSampleCount = 0;
    hasSentShooterSetpoint = false;
    executionTimeoutElapsed = false;
    executionTimer.restart();

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
  double hubX = 0.0;
  double hubY = 0.0;
  double normalizedAngleErrorRad = 0.0;
  boolean shouldShoot = false;

  private boolean shouldUpdateShooterSetpoint(
      double velocityRps, double hoodAngleDeg, double turretAngleDeg, boolean shouldShoot) {
    if (!hasSentShooterSetpoint) {
      return true;
    }

    if (shouldShoot != lastSentShootMode) {
      return true;
    }

    return Math.abs(velocityRps - lastSentVelocityRps) > SHOOTER_SETPOINT_RPS_DEADBAND
        || Math.abs(hoodAngleDeg - lastSentHoodAngleDeg) > SHOOTER_SETPOINT_HOOD_DEG_DEADBAND
        || Math.abs(turretAngleDeg - lastSentTurretAngleDeg) > SHOOTER_SETPOINT_TURRET_DEG_DEADBAND;
  }

  private void cacheLastShooterSetpoint(
      double velocityRps, double hoodAngleDeg, double turretAngleDeg, boolean shouldShoot) {
    hasSentShooterSetpoint = true;
    lastSentVelocityRps = velocityRps;
    lastSentHoodAngleDeg = hoodAngleDeg;
    lastSentTurretAngleDeg = turretAngleDeg;
    lastSentShootMode = shouldShoot;
  }

  @Override
  public void execute() {
    // 1) Gather drivetrain state and derive shooter point kinematics.
    swerveDrivetrain.enableShootPassAndMove();
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

    // v_point = v_center + omega x r
    shooterSpeedX = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * shooterOffsetFieldY;
    shooterSpeedY = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * shooterOffsetFieldX;

    hubX = hubAimPose.getX();
    hubY = hubAimPose.getY();
    distance = Math.hypot(hubX - shooterPoseX, hubY - shooterPoseY);
    time = ShooterCalculator.getShooterFlightTime(distance);

    filteredSpeedX = 0.0;
    filteredSpeedY = 0.0;
    filteredAngleError = 0.0;

    speedXBuffer[bufferIndex] = shooterSpeedX;
    speedYBuffer[bufferIndex] = shooterSpeedY;
    if (validSampleCount < MoveNShootConstants.SHOOT_FILTER_SIZE) {
      validSampleCount++;
    }

    for (int i = 0; i < validSampleCount; i++) {
      filteredSpeedX += speedXBuffer[i];
      filteredSpeedY += speedYBuffer[i];
    }

    filteredSpeedX /= validSampleCount;
    filteredSpeedY /= validSampleCount;

    // 2) Predict where shooter/chassis will be when turret reaches setpoint.
    predictedShooterX = shooterPoseX + filteredSpeedX * TURRET_LOOKAHEAD_SEC;
    predictedShooterY = shooterPoseY + filteredSpeedY * TURRET_LOOKAHEAD_SEC;
    predictedHeading = heading + speeds.omegaRadiansPerSecond * TURRET_LOOKAHEAD_SEC;

    aimX = hubX - (filteredSpeedX * time);
    aimY = hubY - (filteredSpeedY * time);

    robotAngleToHub = Math.atan2(aimY - predictedShooterY, aimX - predictedShooterX);
    rawAngleError = robotAngleToHub - predictedHeading;
    normalizedAngleErrorRad = Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError));

    angleErrorBuffer[bufferIndex] = normalizedAngleErrorRad;

    sumSin = 0.0;
    sumCos = 0.0;
    for (int i = 0; i < validSampleCount; i++) {
      sumSin += Math.sin(angleErrorBuffer[i]);
      sumCos += Math.cos(angleErrorBuffer[i]);
    }
    filteredAngleError = Math.atan2(sumSin, sumCos);

    bufferIndex = (bufferIndex + 1) % MoveNShootConstants.SHOOT_FILTER_SIZE;

    // 3) Solve shooter parameters from filtered motion estimate and predicted aim point.
    shootParams =
        ShooterCalculator.calculateShootingParameters(
            filteredSpeedX,
            filteredSpeedY,
            shooterPoseX,
            shooterPoseY,
            time + TURRET_LOOKAHEAD_SEC);
    velocityRPS = shootParams[0];
    hoodAngle = shootParams[1];
    turretAngleDeg = Math.toDegrees(normalizedAngleErrorRad);

    shouldShoot = theMachine.isShooterReady() || Robot.isSimulation();

    if (!executionTimeoutElapsed && executionTimer.hasElapsed(3.0)) {
      executionTimeoutElapsed = true;
    }

    if (shouldUpdateShooterSetpoint(velocityRPS, hoodAngle, turretAngleDeg, shouldShoot)) {
      if (shouldShoot) {
        if(!executionTimeoutElapsed)
          theMachine.shoot(velocityRPS, hoodAngle, turretAngleDeg);
        else
          theMachine.shootIntakeClosed(velocityRPS, hoodAngle, turretAngleDeg);

      } else {
        theMachine.getReady(velocityRPS, hoodAngle, turretAngleDeg);
      }
      cacheLastShooterSetpoint(velocityRPS, hoodAngle, turretAngleDeg, shouldShoot);
    }

    if (telemetryEnabledEntry.get(false)) {
      SmartDashboard.putNumber(
          "AimShoot/DistanceToHubM", TelemetryConstants.roundTelemetry(distance));
      SmartDashboard.putNumber("AimShoot/FlightTimeSec", TelemetryConstants.roundTelemetry(time));
      SmartDashboard.putNumber(
          "AimShoot/ShooterGoalRps", TelemetryConstants.roundTelemetry(velocityRPS));
      SmartDashboard.putNumber(
          "AimShoot/HoodGoalDeg", TelemetryConstants.roundTelemetry(hoodAngle));
      SmartDashboard.putNumber(
          "AimShoot/TurretGoalDeg", TelemetryConstants.roundTelemetry(turretAngleDeg));
      SmartDashboard.putNumber(
          "AimShoot/AngleErrorDeg",
          TelemetryConstants.roundTelemetry(Math.toDegrees(filteredAngleError)));
      SmartDashboard.putNumber(
          "AimShoot/FilteredSpeedX", TelemetryConstants.roundTelemetry(filteredSpeedX));
      SmartDashboard.putNumber(
          "AimShoot/FilteredSpeedY", TelemetryConstants.roundTelemetry(filteredSpeedY));
      SmartDashboard.putBoolean("AimShoot/ShouldShoot", shouldShoot);
    }

    if (Robot.isSimulation()) {
      aimPose3d.set(new Pose3d(aimX, aimY, Dimensions.HUB_HEIGHT.in(Meters), new Rotation3d()));
      aimOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
      aimAngleErrorEntry.set(Math.toDegrees(filteredAngleError));
    }
  }

  public boolean isExecutionTimeoutElapsed() {
    return executionTimeoutElapsed;
  }

  @Override
  public void end(boolean interrupted) {
    hasSentShooterSetpoint = false;
    executionTimeoutElapsed = false;
    theMachine.idleDeployed();
    swerveDrivetrain.disableShootPassAndMove();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}