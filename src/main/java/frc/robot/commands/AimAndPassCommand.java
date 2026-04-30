// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.TheMachine;
import frc.robot.constants.MoveNShootConstants;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;
import frc.robot.utils.ShooterCalculator;

/**
 * Driver-assist command for passing to lane-specific field targets.
 *
 * <p>Driver keeps manual drive control while turret/hood/flywheel setpoints are solved
 * automatically for the selected pass lane.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndPassCommand extends Command {

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

  private double MaxSpeed =
      MoveNShootConstants.PASS_MAX_SPEED_FRACTION * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate =
      RotationsPerSecond.of(MoveNShootConstants.PASS_MAX_ANGULAR_RATE_RPS).in(RadiansPerSecond);

  private SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private CommandXboxController driverController;
  private CommandSwerveDrivetrain swerveDrivetrain;
  private TheMachine theMachine;

  private Pose2d passAimPose;

  private final double[] speedXBuffer = new double[MoveNShootConstants.PASS_FILTER_SIZE];
  private final double[] speedYBuffer = new double[MoveNShootConstants.PASS_FILTER_SIZE];
  private final double[] angleErrorBuffer = new double[MoveNShootConstants.PASS_FILTER_SIZE];
  private int bufferIndex = 0;
  private int validSampleCount = 0;

  private static final double TURRET_TOLERANCE_DEG =
      ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);
  private static final double TURRET_LOOKAHEAD_SEC = MoveNShootConstants.TURRET_LOOKAHEAD_SEC;
  private static final double PASS_SETPOINT_RPS_DEADBAND = MoveNShootConstants.SETPOINT_RPS_DEADBAND;
  private static final double PASS_SETPOINT_HOOD_DEG_DEADBAND = MoveNShootConstants.SETPOINT_HOOD_DEG_DEADBAND;
  private static final double PASS_SETPOINT_TURRET_DEG_DEADBAND = MoveNShootConstants.SETPOINT_TURRET_DEG_DEADBAND;

  private boolean hasSentPassSetpoint = false;
  private double lastSentVelocityRps = 0.0;
  private double lastSentHoodAngleDeg = 0.0;
  private double lastSentTurretAngleDeg = 0.0;
  private boolean lastSentPassMode = false;

  /** Creates a new AimAndPassCommand. */
  public AimAndPassCommand(
      CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.driverController = joystick;
    this.theMachine = theMachine;

    addRequirements(drivetrain);
    addRequirements(theMachine.getSubsystems());

    passAimPose = Container.isBlue ? PoseConstants.BLUE_PASS_RIGHT : PoseConstants.RED_PASS_RIGHT;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < MoveNShootConstants.PASS_FILTER_SIZE; i++) {
      speedXBuffer[i] = 0.0;
      speedYBuffer[i] = 0.0;
      angleErrorBuffer[i] = 0.0;
    }
    bufferIndex = 0;
    validSampleCount = 0;
    hasSentPassSetpoint = false;

    passAimPose = Container.isBlue ? PoseConstants.BLUE_PASS_RIGHT : PoseConstants.RED_PASS_RIGHT;
  }

  private boolean shouldUpdatePassSetpoint(
      double velocityRps, double hoodAngleDeg, double turretAngleDeg, boolean shouldPass) {
    if (!hasSentPassSetpoint) {
      return true;
    }

    if (shouldPass != lastSentPassMode) {
      return true;
    }

    return Math.abs(velocityRps - lastSentVelocityRps) > PASS_SETPOINT_RPS_DEADBAND
        || Math.abs(hoodAngleDeg - lastSentHoodAngleDeg) > PASS_SETPOINT_HOOD_DEG_DEADBAND
        || Math.abs(turretAngleDeg - lastSentTurretAngleDeg) > PASS_SETPOINT_TURRET_DEG_DEADBAND;
  }

  private void cacheLastPassSetpoint(
      double velocityRps, double hoodAngleDeg, double turretAngleDeg, boolean shouldPass) {
    hasSentPassSetpoint = true;
    lastSentVelocityRps = velocityRps;
    lastSentHoodAngleDeg = hoodAngleDeg;
    lastSentTurretAngleDeg = turretAngleDeg;
    lastSentPassMode = shouldPass;
  }

  private Pose2d robotPose = new Pose2d();
  private ChassisSpeeds speeds = new ChassisSpeeds();
  private double filteredAngleError = 0.0;
  private double filteredSpeedX = 0.0;
  private double filteredSpeedY = 0.0;

  private double velocityRPS = 0.0;
  private double hoodAngle = 0.0;
  private double[] passResult;
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
  double headingSin;
  double headingCos;
  double shooterOffsetFieldX;
  double shooterOffsetFieldY;
  double shooterX;
  double shooterY;
  double shooterSpeedX;
  double shooterSpeedY;
  double predictedShooterX;
  double predictedShooterY;
  double predictedHeading;
  double robotAngleToPass;
  double rawAngleError;
  double normalizedAngleErrorRad;
  double sumSin;
  double sumCos;
  boolean shouldPass;
  double distance;

  double time;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrivetrain.enableShootPassAndMove();

    // 1) Determine current pass target from robot lane side.
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

    shooterX = robotPose.getX() + shooterOffsetFieldX;
    shooterY = robotPose.getY() + shooterOffsetFieldY;
    shooterSpeedX = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * shooterOffsetFieldY;
    shooterSpeedY = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * shooterOffsetFieldX;

    passAimPose = selectPassAimPose(robotPose);

    // Update speed buffers before filtering so newest sample contributes this cycle.
    speedXBuffer[bufferIndex] = shooterSpeedX;
    speedYBuffer[bufferIndex] = shooterSpeedY;
    if (validSampleCount < MoveNShootConstants.PASS_FILTER_SIZE) {
      validSampleCount++;
    }

    filteredSpeedX = 0.0;
    filteredSpeedY = 0.0;
    filteredAngleError = 0.0;
    for (int i = 0; i < validSampleCount; i++) {
      filteredSpeedX += speedXBuffer[i];
      filteredSpeedY += speedYBuffer[i];
    }
    filteredSpeedX /= validSampleCount;
    filteredSpeedY /= validSampleCount;

    predictedShooterX = shooterX + filteredSpeedX * TURRET_LOOKAHEAD_SEC;
    predictedShooterY = shooterY + filteredSpeedY * TURRET_LOOKAHEAD_SEC;
    predictedHeading = heading + speeds.omegaRadiansPerSecond * TURRET_LOOKAHEAD_SEC;

    distance = Math.hypot(passAimPose.getX() - shooterX, passAimPose.getY() - shooterY);
    time = ShooterCalculator.getPassFlightTime(distance);

    aimX = passAimPose.getX() - (filteredSpeedX * time);
    aimY = passAimPose.getY() - (filteredSpeedY * time);

    robotAngleToPass = Math.atan2(aimY - predictedShooterY, aimX - predictedShooterX);
    rawAngleError = robotAngleToPass - predictedHeading;
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

    bufferIndex = (bufferIndex + 1) % MoveNShootConstants.PASS_FILTER_SIZE;

    turretAngleDeg = Math.toDegrees(normalizedAngleErrorRad);

    // 2) Keep manual driving active while assist computes pass turret angle.
    swerveDrivetrain.setControl(
        drive
            .withVelocityX(
                -driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(
                -driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate));

    // 3) Solve pass setpoints and gate feed on shooter readiness.
    passResult = ShooterCalculator.calculatePassParameters(filteredSpeedX, filteredSpeedY, shooterX, shooterY, passAimPose);
    velocityRPS = passResult[0];
    hoodAngle = passResult[1];

    shouldPass = theMachine.isPassReady() || Robot.isSimulation();
    if (shouldUpdatePassSetpoint(velocityRPS, hoodAngle, turretAngleDeg, shouldPass)) {
      if (shouldPass) {
        theMachine.pass(velocityRPS, hoodAngle, turretAngleDeg);
      } else {
        theMachine.getReadyPass(velocityRPS, hoodAngle, turretAngleDeg);
      }
      cacheLastPassSetpoint(velocityRPS, hoodAngle, turretAngleDeg, shouldPass);
    }

    if (Robot.isSimulation()) {
      passOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
      passAngleErrorEntry.set(Math.toDegrees(filteredAngleError));
      passAimPoseXEntry.set(aimX);
      passAimPoseYEntry.set(aimY);
      passAimPoseZEntry.set(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrivetrain.disableShootPassAndMove();
    hasSentPassSetpoint = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
