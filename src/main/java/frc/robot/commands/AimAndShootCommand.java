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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.robot.TheMachine;
import frc.robot.constants.Dimensions;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;
import frc.robot.utils.ShooterCalculator;

/**
 * Driver-assist command for shooting: keeps manual translational control while auto-solving
 * shooter setpoints (flywheel/hood/turret) toward the alliance hub.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAndShootCommand extends Command {
  
  private final DoubleEntry aimAngleErrorEntry = NetworkTableInstance.getDefault()
      .getDoubleTopic("/AIM/AimAngleError").getEntry(0.0);

  private final StructPublisher<Pose3d> aimPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("/AIM/AimPose3d", Pose3d.struct)
    .publish();
  private final BooleanEntry aimOnTargetEntry = NetworkTableInstance.getDefault()
    .getBooleanTopic("/AIM/AimOnTarget").getEntry(false);

  

  private double MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.35).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private static final Translation2d SHOOTER_CENTER_OF_ROTATION =
      new Translation2d(
          TheMachineConstants.SHOOTER_ROTATION_AXIS.getX(),
          TheMachineConstants.SHOOTER_ROTATION_AXIS.getY());

  private static final Transform2d SHOOTER_OFFSET_FROM_ROBOT = new Transform2d(
        TheMachineConstants.SHOOTER_ROTATION_AXIS.getX(),
        TheMachineConstants.SHOOTER_ROTATION_AXIS.getY(),
        Rotation2d.kZero
    );

  private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withCenterOfRotation(SHOOTER_CENTER_OF_ROTATION); // Use open-loop control for drive motors

  private CommandXboxController driverController;
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

  private SlewRateLimiter joyXSlewLimiter = new SlewRateLimiter(2.5); 
  private SlewRateLimiter joyYSlewLimiter = new SlewRateLimiter(2.5);


  /** Creates a new AimAndShootCommand. */
  public AimAndShootCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick, TheMachine theMachine) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.driverController = joystick;
    this.theMachine = theMachine;

    addRequirements(drivetrain);
    addRequirements(theMachine.getSubsystems());
    
    if(Container.isBlue)
    {
      hubAimPose = PoseConstants.BLUE_HUB_AIM_POSE;
    }
    else
    {
      hubAimPose = PoseConstants.RED_HUB_AIM_POSE;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Clear moving average buffers
    for (int i = 0; i < FILTER_SIZE; i++) {
      speedXBuffer[i] = 0.0;
      speedYBuffer[i] = 0.0;
      angleErrorBuffer[i] = 0.0;
    }
    bufferIndex = 0;
    validSampleCount = 0;

    // Re-resolve alliance hub target in case DS alliance changed while disabled.
    if(Container.isBlue)
    {
      hubAimPose = PoseConstants.BLUE_HUB_AIM_POSE;
    }
    else
    {
      hubAimPose = PoseConstants.RED_HUB_AIM_POSE;
    }
  }

  private Pose2d robotPose = new Pose2d();
  private Pose2d shooterPose = new Pose2d();
  double robotAngleToHub =  0;

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

  Translation2d shooterOffsetField;

  double filteredSpeedX = 0.0, filteredSpeedY = 0.0, filteredAngleError = 0.0;
  private static final double TURRET_TOLERANCE_DEG = ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);
  private static final double TURRET_LOOKAHEAD_SEC = 0.04;

  double rawAngleError = 0.0;
  double shooterSpeedX = 0.0;
  double shooterSpeedY = 0.0;
  double predictedHeading = 0.0;
  double predictedShooterX = 0.0;
  double predictedShooterY = 0.0;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 1) Gather drivetrain state and derive shooter point kinematics.
    robotPose = swerveDrivetrain.getPose();
    shooterPose = robotPose.transformBy(SHOOTER_OFFSET_FROM_ROBOT);
    speeds = swerveDrivetrain.getFieldSpeeds();
    heading = robotPose.getRotation().getRadians();

    // Convert robot-relative CoR offset to field frame and compute shooter-point velocity:
    // v_point = v_center + omega x r
    shooterOffsetField = SHOOTER_CENTER_OF_ROTATION.rotateBy(robotPose.getRotation());
    shooterSpeedX = speeds.vxMetersPerSecond - speeds.omegaRadiansPerSecond * shooterOffsetField.getY();
    shooterSpeedY = speeds.vyMetersPerSecond + speeds.omegaRadiansPerSecond * shooterOffsetField.getX();

    distance = hubAimPose.getTranslation().getDistance(shooterPose.getTranslation());
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
    for (int i = 0; i < validSampleCount; i++) {
      filteredSpeedX += speedXBuffer[i];
      filteredSpeedY += speedYBuffer[i];
    }

    filteredSpeedX /= validSampleCount;
    filteredSpeedY /= validSampleCount;

    // 2) Predict where shooter/chassis will be when turret reaches setpoint.
    predictedShooterX = shooterPose.getX() + filteredSpeedX * TURRET_LOOKAHEAD_SEC;
    predictedShooterY = shooterPose.getY() + filteredSpeedY * TURRET_LOOKAHEAD_SEC;
    predictedHeading = heading + speeds.omegaRadiansPerSecond * TURRET_LOOKAHEAD_SEC;

    aimX = hubAimPose.getX() - (filteredSpeedX * time);
    aimY = hubAimPose.getY() - (filteredSpeedY * time);

    robotAngleToHub = Math.atan2(aimY - predictedShooterY, aimX - predictedShooterX);
    rawAngleError = robotAngleToHub - predictedHeading;
    // Normalize angle error to [-π, π]
    rawAngleError = Math.atan2(Math.sin(rawAngleError), Math.cos(rawAngleError));
    
    // Update angle buffer and compute wrap-safe filtered angle.
    angleErrorBuffer[bufferIndex] = rawAngleError;

    double sumSin = 0.0;
    double sumCos = 0.0;
    for (int i = 0; i < validSampleCount; i++) {
      sumSin += Math.sin(angleErrorBuffer[i]);
      sumCos += Math.cos(angleErrorBuffer[i]);
    }
    filteredAngleError = Math.atan2(sumSin, sumCos);

    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;

    // 3) Run normal driver translation/rotation while assist computes shooter setpoints.
    swerveDrivetrain.setControl(
        drive.withVelocityX(-joyYSlewLimiter.calculate(driverController.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-joyXSlewLimiter.calculate(driverController.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
      .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
    );

    // 4) Solve shooter parameters from filtered motion estimate and predicted aim point.
    shootParams = ShooterCalculator.calculateShootingParameters(filteredSpeedX, filteredSpeedY, robotPose, time);
    velocityRPS = shootParams[0];
    hoodAngle = shootParams[1];
    turretAngleDeg = Math.toDegrees(Math.atan2(Math.sin(robotAngleToHub - predictedHeading), Math.cos(robotAngleToHub - predictedHeading)));

    // Feed only once shooter is ready; otherwise stay in spin-up state.
    if(theMachine.isShooterReady()) {
      theMachine.shoot(velocityRPS, hoodAngle, turretAngleDeg);
    } else {
      theMachine.getReady(velocityRPS, hoodAngle, turretAngleDeg);
  }

  // Publish simulated aiming telemetry for dashboards/3D overlays.
    if(Robot.isSimulation()) {
      aimPosePublisher.set(new Pose3d(aimX, aimY, Dimensions.HUB_HEIGHT.in(Meters), new Rotation3d(0, 0, 0)));
      aimOnTargetEntry.set(Math.abs(turretAngleDeg) <= TURRET_TOLERANCE_DEG);
      aimAngleErrorEntry.set(Math.toDegrees(filteredAngleError));
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
