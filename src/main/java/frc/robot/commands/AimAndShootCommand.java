// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.TheMachine;
import frc.robot.constants.Dimensions;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.ShooterCalculator;

/**
 * Driver-assist command for shooting: keeps manual translational control while auto-solving shooter
 * setpoints (flywheel/hood/turret) toward the alliance hub.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootCommand extends Command {

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

  private double MaxSpeed =
      DriveConstants.AIM_MAX_SPEED_FRACTION * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate =
      RotationsPerSecond.of(DriveConstants.AIM_MAX_ANGULAR_RATE_RPS).in(RadiansPerSecond);

  private static final Translation2d SHOOTER_CENTER_OF_ROTATION =
      new Translation2d(
          TheMachineConstants.SHOOTER_ROTATION_AXIS.getX(),
          TheMachineConstants.SHOOTER_ROTATION_AXIS.getY());

  private SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withCenterOfRotation(
              SHOOTER_CENTER_OF_ROTATION); // Use open-loop control for drive motors

  private CommandXboxController driverController;
  private CommandSwerveDrivetrain swerveDrivetrain;
  private TheMachine theMachine;

  private Pose2d hubAimPose;

  // Moving average buffers for noise reduction
  private final double[] speedXBuffer = new double[DriveConstants.AIM_FILTER_SIZE];
  private final double[] speedYBuffer = new double[DriveConstants.AIM_FILTER_SIZE];
  private final double[] angleErrorBuffer = new double[DriveConstants.AIM_FILTER_SIZE];
  private int bufferIndex = 0;
  private int validSampleCount = 0;

  private SlewRateLimiter joyXSlewLimiter = new SlewRateLimiter(DriveConstants.AIM_TRANSLATION_SLEW_RATE);
  private SlewRateLimiter joyYSlewLimiter = new SlewRateLimiter(DriveConstants.AIM_TRANSLATION_SLEW_RATE);

  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand(
      CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.driverController = joystick;
    this.theMachine = theMachine;

    addRequirements(drivetrain);
    addRequirements(theMachine.getSubsystems());

    hubAimPose = AllianceUtil.getHubAimPose();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear moving average buffers
    for (int i = 0; i < DriveConstants.AIM_FILTER_SIZE; i++) {
      speedXBuffer[i] = 0.0;
      speedYBuffer[i] = 0.0;
      angleErrorBuffer[i] = 0.0;
    }
    bufferIndex = 0;
    validSampleCount = 0;
    hasSentShooterSetpoint = false;

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
  private static final double TURRET_LOOKAHEAD_SEC = 0.1;
  private static final double SHOOTER_SETPOINT_RPS_DEADBAND = 0.05;
  private static final double SHOOTER_SETPOINT_HOOD_DEG_DEADBAND = 0.05;
  private static final double SHOOTER_SETPOINT_TURRET_DEG_DEADBAND = 0.10;

  private boolean hasSentShooterSetpoint = false;
  private double lastSentVelocityRps = 0.0;
  private double lastSentHoodAngleDeg = 0.0;
  private double lastSentTurretAngleDeg = 0.0;
  private boolean lastSentShootMode = false;

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
  double leftY = 0.0;
  double leftX = 0.0;
  double rightX = 0.0;
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

    hubX = hubAimPose.getX();
    hubY = hubAimPose.getY();
    distance = Math.hypot(hubX - shooterPoseX, hubY - shooterPoseY);
    time = ShooterCalculator.getShooterFlightTime(distance);

    filteredSpeedX = 0.0;
    filteredSpeedY = 0.0;
    filteredAngleError = 0.0;

    // Update speed buffers before filtering so newest sample contributes this cycle.
    speedXBuffer[bufferIndex] = shooterSpeedX;
    speedYBuffer[bufferIndex] = shooterSpeedY;
    if (validSampleCount < DriveConstants.AIM_FILTER_SIZE) {
      validSampleCount++;
    }

    // Calculate filtered (smoothed) values
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
    // Normalize angle error to [-π, π]
    normalizedAngleErrorRad = Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError));

    // Update angle buffer and compute wrap-safe filtered angle.
    angleErrorBuffer[bufferIndex] = normalizedAngleErrorRad;

    sumSin = 0.0;
    sumCos = 0.0;
    for (int i = 0; i < validSampleCount; i++) {
      sumSin += Math.sin(angleErrorBuffer[i]);
      sumCos += Math.cos(angleErrorBuffer[i]);
    }
    filteredAngleError = Math.atan2(sumSin, sumCos);

    bufferIndex = (bufferIndex + 1) % DriveConstants.AIM_FILTER_SIZE;

    // 3) Run normal driver translation/rotation while assist computes shooter setpoints.
    leftY = driverController.getLeftY();
    leftX = driverController.getLeftX();
    rightX = driverController.getRightX();
    swerveDrivetrain.setControl(
        drive
            .withVelocityX(
                -joyYSlewLimiter.calculate(leftY)
                    * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(
                -joyXSlewLimiter.calculate(leftX) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-rightX * MaxAngularRate));

    // 4) Solve shooter parameters from filtered motion estimate and predicted aim point.
    shootParams =
        ShooterCalculator.calculateShootingParameters(
            filteredSpeedX,
            filteredSpeedY,
            shooterPoseX,
            shooterPoseY,
            time);
    velocityRPS = shootParams[0];
    hoodAngle = shootParams[1];
    turretAngleDeg = Math.toDegrees(normalizedAngleErrorRad);

    // Feed only once shooter is ready; otherwise stay in spin-up state.
    shouldShoot = theMachine.isShooterReady() || Robot.isSimulation();
    if (shouldUpdateShooterSetpoint(velocityRPS, hoodAngle, turretAngleDeg, shouldShoot)) {
      if (shouldShoot) {
        theMachine.shoot(velocityRPS, hoodAngle, turretAngleDeg);
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

    // Publish simulated aiming telemetry for dashboards/3D overlays.
    if (Robot.isSimulation()) {
      aimPose3d.set(new Pose3d(aimX, aimY, Dimensions.HUB_HEIGHT.in(Meters), new Rotation3d()));
      aimOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
      aimAngleErrorEntry.set(Math.toDegrees(filteredAngleError));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hasSentShooterSetpoint = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
