package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.States.TheMachineStates.TheMachineState;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.LEDController;
import frc.robot.utils.ShooterCalculator;
import java.util.function.Supplier;

/**
 * High-level mechanism coordinator ("state machine") for shooter, feeder, hopper, and intake.
 *
 * <p>This class defines the intent-level actions (intake, shoot, pass, idle...), then delegates
 * low-level motor behavior to the subsystems.
 */
public class TheMachine {

  private final ShooterSubsystem shooterSubsystem;
  private final HopperSubsystem hopperSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final SubsystemBase[] subsystems;

  // Current orchestrated machine mode.
  private TheMachineState state = TheMachineState.IDLE_RETRACTED;

  // Optional intake mode that changes extension behavior.
  private boolean intakeWithOffset = false;

  private Supplier<Pose2d> robotPose2dSupplier;
  private Pose2d goalHubPose;
  private final LEDController ledController;

  private enum LedState {
    OFF,
    ZERO,
    IDLE_RETRACTED,
    IDLE_DEPLOYED,
    INTAKE,
    GET_READY,
    READY_TO_SHOOT,
    GET_READY_PASS,
    READY_TO_PASS,
    REVERSE,
    TEST,
    MANUAL_OVERRIDE,
    UNKNOWN
  }

  private LedState currentLedState = LedState.UNKNOWN;

  public TheMachine(
      ShooterSubsystem shooterSubsystem,
      HopperSubsystem hopperSubsystem,
      IntakeSubsystem intakeSubsystem,
      FeederSubsystem feederSubsystem,
      Supplier<Pose2d> robotPose2dSupplier) {
    this.shooterSubsystem = shooterSubsystem;
    this.hopperSubsystem = hopperSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.subsystems =
        new SubsystemBase[] {
          this.shooterSubsystem, this.feederSubsystem, this.hopperSubsystem, this.intakeSubsystem
        };

    this.robotPose2dSupplier = robotPose2dSupplier;
    this.ledController =
        new LEDController(TheMachineConstants.LED_PWM_PORT, TheMachineConstants.LED_STRIP_LENGTH);

    // Choose the alliance-correct hub pose for aiming calculations.
    goalHubPose = AllianceUtil.getHubAimPose();
  }

  private double goalTurretAngleForHub;
  private Pose2d robotPose2d;

  double shooterPoseX;
  double shooterPoseY;
  double robotHeadingRad;
  double shooterToHubAngleRad;

  /** Aim turret toward the hub using current field pose, without enabling feed/shoot actions. */
  public void setTurretAngleToHubWithoutShooting() {
    shooterSubsystem.setTurretAngleDegrees(getTurretAngleToHub());
  }

  public double getTurretAngleToHub() {
    goalHubPose = AllianceUtil.getHubAimPose();
    robotPose2d = robotPose2dSupplier.get();
    robotHeadingRad = robotPose2d.getRotation().getRadians();
    shooterPoseX = ShooterCalculator.getShooterXFromRobotState(robotPose2d.getX(), robotHeadingRad);
    shooterPoseY = ShooterCalculator.getShooterYFromRobotState(robotPose2d.getY(), robotHeadingRad);

    shooterToHubAngleRad =
        Math.atan2(goalHubPose.getY() - shooterPoseY, goalHubPose.getX() - shooterPoseX);

    goalTurretAngleForHub =
        Math.toDegrees(
            Math.atan2(
                Math.sin(shooterToHubAngleRad - robotHeadingRad),
                Math.cos(shooterToHubAngleRad - robotHeadingRad)));

    return goalTurretAngleForHub;
  }

  private final double[] hubShootValues = new double[2]; // [0]=flywheelRps, [1]=hoodAngleDeg

  /**
   * Returns calculated shooter setpoints for hub targeting. Index 0 = flywheel RPS, index 1 = hood
   * angle degrees.
   */
  public double[] getShooterValuesForHub() {
    goalHubPose = AllianceUtil.getHubAimPose();
    robotPose2d = robotPose2dSupplier.get();
    robotHeadingRad = robotPose2d.getRotation().getRadians();
    shooterPoseX = ShooterCalculator.getShooterXFromRobotState(robotPose2d.getX(), robotHeadingRad);
    shooterPoseY = ShooterCalculator.getShooterYFromRobotState(robotPose2d.getY(), robotHeadingRad);

    double[] shootingParams =
        ShooterCalculator.calculateShootingParameters(0.0, 0.0, shooterPoseX, shooterPoseY, 0.0);
    hubShootValues[0] = shootingParams[0];
    hubShootValues[1] = shootingParams[1];
    return hubShootValues;
  }

  public double getHoodAngleForHub() {
    return getShooterValuesForHub()[1];
  }

  /** Reset mechanisms to a known zero-like safe state. */
  public void zero() {
    shooterSubsystem.zero();
    feederSubsystem.zero();
    hopperSubsystem.zero();
    intakeSubsystem.close();
    state = TheMachineState.ZERO;
  }

  /** Intake stowed, shooter resting, feeder reversed to prevent accidental feed. */
  public void idleRetracted() {
    shooterSubsystem.rest();
    feederSubsystem.zero();
    hopperSubsystem.zero();
    intakeSubsystem.close();
    state = TheMachineState.IDLE_RETRACTED;
  }

  /** Intake deployed but otherwise idle. */
  public void idleDeployed() {
    shooterSubsystem.rest();
    feederSubsystem.zero();
    hopperSubsystem.push();
    intakeSubsystem.feed();
    state = TheMachineState.IDLE_DEPLOYED;
  }

  /** Neutral between-state used by command transitions. */
  public void idle() {
    shooterSubsystem.rest();
    feederSubsystem.zero();
    hopperSubsystem.zero();
    intakeSubsystem.idleBetween();
    state = TheMachineState.IDLE;
  }

  /** Run intake pipeline (intake + hopper path) while shooter stays at rest. */
  public void intake() {
    shooterSubsystem.rest();
    feederSubsystem.zero();
    hopperSubsystem.zero();
    if (intakeWithOffset) {
      intakeSubsystem.intakeWithOffset();
    } else {
      intakeSubsystem.intake();
    }
    state = TheMachineState.INTAKE;
  }

  /** Spin up for shooting without feeding into flywheels yet. */
  public void getReady(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
    shooterSubsystem.shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    feederSubsystem.feedGetReady();
    hopperSubsystem.idle();
    intakeSubsystem.intake();
    state = TheMachineState.GET_READY;
  }

  public void getReady(double velocityRPS, double hoodAngleRotations) {
    getReady(velocityRPS, hoodAngleRotations, 0.0);
  }

  /** Shoot at requested setpoints. */
  public void shoot(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
    shooterSubsystem.shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    feederSubsystem.feed();
    hopperSubsystem.feed();
    intakeSubsystem.intake();
    state = TheMachineState.SHOOT;
  }

  public void shoot(double velocityRPS, double hoodAngleRotations) {
    shoot(velocityRPS, hoodAngleRotations, 0.0);
  }

  /** Spin up for passing without feeding yet. */
  public void getReadyPass(
      double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
    shooterSubsystem.pass(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    feederSubsystem.feedGetReady();
    hopperSubsystem.idle();
    intakeSubsystem.intake();
    state = TheMachineState.GET_READY_PASS;
  }

  public void getReadyPass(double velocityRPS, double hoodAngleRotations) {
    getReadyPass(velocityRPS, hoodAngleRotations, 0.0);
  }

  /** Pass mode feeds cargo at pass setpoints. */
  public void pass(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
    shooterSubsystem.pass(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    feederSubsystem.feed();
    hopperSubsystem.feed();
    intakeSubsystem.intake();
    state = TheMachineState.PASS;
  }

  public void pass(double velocityRPS, double hoodAngleRotations) {
    pass(velocityRPS, hoodAngleRotations, 0.0);
  }

  /** Reverse all cargo-path motors to clear jams. */
  public void reverse() {
    shooterSubsystem.zero();
    feederSubsystem.reverse();
    hopperSubsystem.reverse();
    intakeSubsystem.reverse();
    state = TheMachineState.REVERSE;
  }

  /** Open-loop/manual test mode. */
  public void test() {
    shooterSubsystem.test();
    feederSubsystem.feed();
    hopperSubsystem.feed();
    intakeSubsystem.feed();
    state = TheMachineState.TEST;
  }

  /** Open-loop/manual test mode with explicit per-subsystem setpoints. */
  public void test(
      double shooterFlywheelRps,
      double shooterHoodRot,
      double shooterTurretDeg,
      double feederBeltRps,
      double feederFeedRps,
      double hopperRps,
      double intakeRps,
      double intakeExtensionMeters) {
    shooterSubsystem.test(shooterFlywheelRps, shooterHoodRot, shooterTurretDeg);
    feederSubsystem.test(feederBeltRps, feederFeedRps);
    hopperSubsystem.test(hopperRps);
    intakeSubsystem.test(intakeRps, intakeExtensionMeters);
    state = TheMachineState.TEST;
  }

  public void testWithTurretToHub(
      double shooterFlywheelRps,
      double shooterHoodRot,
      double feederBeltRps,
      double feederFeedRps,
      double hopperRps,
      double intakeRps,
      double intakeExtensionMeters) {
    shooterSubsystem.test(shooterFlywheelRps, shooterHoodRot, getTurretAngleToHub());
    feederSubsystem.test(feederBeltRps, feederFeedRps);
    hopperSubsystem.test(hopperRps);
    intakeSubsystem.test(intakeRps, intakeExtensionMeters);
    state = TheMachineState.TEST;
  }

  public void testWithTurretToHubAndHoodCalculated(
      double shooterFlywheelRps,
      double feederBeltRps,
      double feederFeedRps,
      double hopperRps,
      double intakeRps,
      double intakeExtensionMeters) {
    shooterSubsystem.test(shooterFlywheelRps, getTurretAngleToHub());
    feederSubsystem.test(feederBeltRps, feederFeedRps);
    hopperSubsystem.test(hopperRps);
    intakeSubsystem.test(intakeRps, intakeExtensionMeters);
    state = TheMachineState.TEST;
  }

  public void shootTest(double shooterFlywheelRps, double shooterHoodRot, double shooterTurretDeg) {
    shooterSubsystem.shoot(shooterFlywheelRps, shooterHoodRot, shooterTurretDeg);
    if (shooterSubsystem.isReadyToShoot() || Robot.isSimulation()) {
      feederSubsystem.feed();
      hopperSubsystem.feed();
      intakeSubsystem.intake();
    } else {
      feederSubsystem.feedGetReady();
      hopperSubsystem.idle();
      intakeSubsystem.intake();
    }
    state = TheMachineState.TEST;
  }

  public void changeIntakeMode() {
    intakeWithOffset = !intakeWithOffset;
  }

  public boolean isIntakeWithOffset() {
    return intakeWithOffset;
  }

  public boolean isState(TheMachineState state) {
    return this.state == state;
  }

  public boolean isShooterReady() {
    return shooterSubsystem.isReadyToShoot();
  }

  public boolean isPassReady() {
    return shooterSubsystem.isReadyToPass();
  }

  public boolean isIntakeHomed() {
    return intakeSubsystem.isIntakeHomed();
  }

  public boolean isHomed() {
    return isIntakeHomed();
  }

  public boolean isAbleToIntake() {
    return state == TheMachineState.INTAKE
        || state == TheMachineState.GET_READY_PASS
        || state == TheMachineState.PASS
        || state == TheMachineState.GET_READY
        || state == TheMachineState.SHOOT;
  }

  private StructPublisher<Pose3d> intakePosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Sim/IntakePose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> shooterPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Sim/ShooterPose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> hoodPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic("Sim/HoodPose", Pose3d.struct).publish();

  public void calculateSubsystemPoses() {
    // Intake extension pose from extension distance + fixed extension angle.
    double intakeExtensionMeters = intakeSubsystem.getIntakeExtensionMeters();
    double intakeExtensionAngleRad =
        Math.toRadians(TheMachineConstants.INTAKE_EXTENSION_ANGLE_DEGREES);
    double intakeExtensionX = intakeExtensionMeters * Math.cos(intakeExtensionAngleRad);
    double intakeExtensionZ = intakeExtensionMeters * Math.sin(intakeExtensionAngleRad);

    SmartDashboard.putNumber(
        "IntakeExtensionMeters", TelemetryConstants.roundTelemetry(intakeExtensionMeters));
    SmartDashboard.putNumber(
        "IntakeExtensionAngleDeg",
        TelemetryConstants.roundTelemetry(TheMachineConstants.INTAKE_EXTENSION_ANGLE_DEGREES));

    Pose3d intakePose =
        TheMachineConstants.INTAKE_RETRACTED_POSE.plus(
            new Transform3d(intakeExtensionX, 0, intakeExtensionZ, new Rotation3d(0, 0, 0)));

    // Turret yaw + hood pitch are reported as independent joints for visualization.
    double turretYawRad = Math.toRadians(shooterSubsystem.getTurretAngleDegrees());
    double hoodPitchRad = shooterSubsystem.getHoodPosition() * 2.0 * Math.PI;
    Pose3d shooterZeroPose = TheMachineConstants.SHOOTER_ZERO_POSE;
    Pose3d hoodZeroPose = TheMachineConstants.HOOD_RETRACTED_POSE;

    Pose3d shooterPose =
        new Pose3d(
            shooterZeroPose.getTranslation(),
            shooterZeroPose.getRotation().plus(new Rotation3d(0, 0, turretYawRad)));

    Transform3d shooterToHoodAtZero = new Transform3d(shooterZeroPose, hoodZeroPose);
    Pose3d hoodYawPose = shooterPose.plus(shooterToHoodAtZero);
    Pose3d hoodPose =
        hoodYawPose.rotateAround(hoodYawPose.getTranslation(), new Rotation3d(hoodPitchRad, 0, 0));

    SmartDashboard.putNumber(
        "Shooter/TurretPoseDeg", TelemetryConstants.roundTelemetry(Math.toDegrees(turretYawRad)));
    SmartDashboard.putNumber(
        "Shooter/HoodPoseDeg", TelemetryConstants.roundTelemetry(Math.toDegrees(hoodPitchRad)));

    intakePosePublisher.set(intakePose);
    shooterPosePublisher.set(shooterPose);
    hoodPosePublisher.set(hoodPose);
  }

  /**
   * @deprecated Use {@link #calculateSubsystemPoses()}.
   */
  @Deprecated
  public void calculateSubsytemPoses() {
    calculateSubsystemPoses();
  }

  public void publishTelemetry() {
    shooterSubsystem.publishTelemetry();
    feederSubsystem.publishTelemetry();
    hopperSubsystem.publishTelemetry();
    intakeSubsystem.publishTelemetry();

    SmartDashboard.putString("TheMachine/State", state.toString());
    SmartDashboard.putString("TheMachine/LedState", currentLedState.toString());
    SmartDashboard.putBoolean(
        "TheMachine/ManualOverrideEnabled", shooterSubsystem.isManualOverrideEnabled());
    SmartDashboard.putBoolean("TheMachine/IntakeHomed", isIntakeHomed());
    SmartDashboard.putBoolean("TheMachine/Homed", isHomed());
  }

  private void setLedState(LedState desiredLedState) {
    if (desiredLedState == currentLedState) {
      return;
    }

    currentLedState = desiredLedState;
    switch (desiredLedState) {
      case OFF:
        ledController.zeroAll();
        break;
      case ZERO:
        ledController.setAll(255, 255, 255); // White
        break;
      case IDLE_RETRACTED:
        ledController.setAll(0, 0, 255); // Blue
        break;
      case IDLE_DEPLOYED:
        ledController.setAll(0, 180, 255); // Cyan
        break;
      case INTAKE:
        ledController.setAll(255, 120, 0); // Orange
        break;
      case GET_READY:
        ledController.setAll(160, 0, 255); // Purple
        break;
      case READY_TO_SHOOT:
        ledController.setAll(0, 255, 0); // Green
        break;
      case GET_READY_PASS:
        ledController.setAll(255, 0, 255); // Magenta
        break;
      case READY_TO_PASS:
        ledController.setAll(0, 255, 120); // Mint
        break;
      case REVERSE:
        ledController.setAll(255, 0, 0); // Red
        break;
      case TEST:
        ledController.setAll(255, 255, 0); // Yellow
        break;
      case MANUAL_OVERRIDE:
        ledController.setAll(255, 0, 80); // Pink
        break;
      case UNKNOWN:
      default:
        ledController.setAll(40, 40, 40); // Dim white fallback
        break;
    }
  }

  private void updateLedsForCurrentState() {
    if (shooterSubsystem.isManualOverrideEnabled()) {
      setLedState(LedState.MANUAL_OVERRIDE);
      return;
    }

    switch (state) {
      case ZERO:
        setLedState(LedState.ZERO);
        break;
      case IDLE_RETRACTED:
        setLedState(LedState.IDLE_RETRACTED);
        break;
      case IDLE_DEPLOYED:
        setLedState(LedState.IDLE_DEPLOYED);
        break;
      case IDLE:
        setLedState(LedState.OFF);
        break;
      case INTAKE:
        setLedState(LedState.INTAKE);
        break;
      case GET_READY:
      case SHOOT:
        setLedState(isShooterReady() ? LedState.READY_TO_SHOOT : LedState.GET_READY);
        break;
      case GET_READY_PASS:
      case PASS:
        setLedState(isShooterReady() ? LedState.READY_TO_PASS : LedState.GET_READY_PASS);
        break;
      case REVERSE:
        setLedState(LedState.REVERSE);
        break;
      case TEST:
        setLedState(LedState.TEST);
        break;
      case NONE:
      default:
        setLedState(LedState.UNKNOWN);
        break;
    }
  }

  public SubsystemBase[] getSubsystems() {
    return subsystems;
  }

  public boolean isManualOverrideEnabled() {
    return shooterSubsystem.isManualOverrideEnabled();
  }

  public void machinePeriodic() {
    updateLedsForCurrentState();

    // Manual override disables automatic hub-facing behavior.
    if (!shooterSubsystem.isManualOverrideEnabled()) {

      // In passive/intake states, keep turret automatically oriented toward hub.
      if (state == TheMachineState.INTAKE
          || state == TheMachineState.IDLE
          || state == TheMachineState.IDLE_DEPLOYED
          || state == TheMachineState.IDLE_RETRACTED) {
        setTurretAngleToHubWithoutShooting();
      }
      return;
    }
  }
}
