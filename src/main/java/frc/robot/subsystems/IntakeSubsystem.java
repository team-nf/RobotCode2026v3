// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TelemetryConstants;

/**
 * Controls intake rollers and intake arm extension.
 *
 * <p>This subsystem owns both hardware and simulation models for the intake mechanism.
 */
public class IntakeSubsystem extends SubsystemBase {

  // Stop intake-arm SysId slightly before hard end offsets.
  private static final double INTAKE_ARM_SYSID_LIMIT_MARGIN_METERS = 0.010; // 10 mm
  private static final double INTAKE_DEPLOYED_THRESHOLD_METERS =
      IntakeConstants.INTAKE_EXTENSION_DEPLOYED_METERS
          - IntakeConstants.INTAKE_EXTENSION_ALLOWABLE_ERROR_METERS;
  private static final double INTAKE_RETRACTED_THRESHOLD_METERS =
      IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS
          + IntakeConstants.INTAKE_EXTENSION_ALLOWABLE_ERROR_METERS;
  private static final double INTAKE_ARM_ZEROING_REVERSE_OUTPUT = -0.12;
  private static final double INTAKE_ARM_ZEROING_STALL_VELOCITY_RPS = 0.05;
  private static final double INTAKE_ARM_ZEROING_STALL_DEBOUNCE_SEC = 0.20;
  private static final double INTAKE_ARM_ZEROING_TIMEOUT_SEC = 2.0;

  private TalonFX intakeMotor;
  private TalonFX intakeMotor2;
  private TalonFX intakeArmMotor;

  private final VelocityVoltage intakeVelocityControl;
  private final PositionVoltage intakeArmPositionControl;

  private final StatusSignal<?> intakeVelocitySignal;
  private final StatusSignal<?> intakeCurrentSignal;
  private final StatusSignal<?> intakeVoltageSignal;
  private final StatusSignal<?> intake2VelocitySignal;
  private final StatusSignal<?> intake2CurrentSignal;
  private final StatusSignal<?> intake2VoltageSignal;
  private final StatusSignal<?> intakeArmPositionSignal;
  private final StatusSignal<?> intakeArmVelocitySignal;
  private final StatusSignal<?> intakeArmCurrentSignal;
  private final StatusSignal<?> intakeArmVoltageSignal;

  private double intakeGoalVelocity;
  private double intakeGoalExtensionMeters;
  private double intakeTestRPM;
  private double intakeTestExtensionMeters;
  private boolean intakeHomed = false;
  private boolean intakeHardstopZeroingActive = false;
  private boolean intakeHardstopZeroingComplete = false;
  private boolean intakeHardstopZeroingSucceeded = false;
  private double intakeHardstopZeroingStartSec = 0.0;
  private double intakeHardstopStallStartSec = -1.0;

  private final VoltageOut intakeRollerSysIdControl;
  private final VoltageOut intakeArmSysIdControl;
  private final SysIdRoutine intakeRollerSysIdRoutine;
  private final SysIdRoutine intakeArmSysIdRoutine;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_ROLLER_PRIMARY_MOTOR_ID);
    intakeMotor2 = new TalonFX(IntakeConstants.INTAKE_ROLLER_SECONDARY_MOTOR_ID);
    intakeArmMotor = new TalonFX(IntakeConstants.INTAKE_LINEAR_MOTOR_ID);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor.getConfigurator().apply(IntakeConstants.INTAKE_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply intake motor configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeMotor2.getConfigurator().apply(IntakeConstants.INTAKE_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply intake motor 2 configs, error code: " + status.toString());
    }

    intakeMotor2.setControl(new Follower(intakeMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = intakeArmMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply intake arm motor configs, error code: " + status.toString());
    }

    // On real hardware, seed extension encoder to known retracted position at boot.
    if (Robot.isReal()) {
      intakeArmMotor.setPosition(
          IntakeConstants.extensionMetersToMotorRotations(
              IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS));
    }
    intakeHomed = true;

    intakeVelocityControl = IntakeConstants.INTAKE_VELOCITY_CONTROL.clone();
    intakeArmPositionControl = IntakeConstants.INTAKE_ARM_POSITION_CONTROL.clone();

    intakeVelocitySignal = intakeMotor.getVelocity(false);
    intakeCurrentSignal = intakeMotor.getStatorCurrent(false);
    intakeVoltageSignal = intakeMotor.getMotorVoltage(false);

    intake2VelocitySignal = intakeMotor2.getVelocity(false);
    intake2CurrentSignal = intakeMotor2.getStatorCurrent(false);
    intake2VoltageSignal = intakeMotor2.getMotorVoltage(false);

    intakeArmPositionSignal = intakeArmMotor.getPosition(false);
    intakeArmVelocitySignal = intakeArmMotor.getVelocity(false);
    intakeArmCurrentSignal = intakeArmMotor.getStatorCurrent(false);
    intakeArmVoltageSignal = intakeArmMotor.getMotorVoltage(false);

    intakeRollerSysIdControl = new VoltageOut(0).withEnableFOC(false);
    intakeArmSysIdControl = new VoltageOut(0).withEnableFOC(false);
    intakeRollerSysIdRoutine =
        createSysIdRoutine("intake/Roller", intakeMotor, intakeRollerSysIdControl);
    intakeArmSysIdRoutine = createSysIdRoutine("intake/Arm", intakeArmMotor, intakeArmSysIdControl);
  }

  private SysIdRoutine createSysIdRoutine(
      String logPrefix, TalonFX motor, VoltageOut voltageControl) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString(logPrefix + "_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
              motor.setControl(voltageControl.withOutput(output.in(Volts)));
              SignalLogger.writeDouble(
                  logPrefix + "_Voltage", motor.getMotorVoltage().getValueAsDouble());
              SignalLogger.writeDouble(
                  logPrefix + "_Position", motor.getPosition().getValueAsDouble());
              SignalLogger.writeDouble(
                  logPrefix + "_Velocity", motor.getVelocity().getValueAsDouble());
            },
            log ->
                log.motor(logPrefix)
                    .voltage(Volts.of(motor.getMotorVoltage().getValueAsDouble()))
                    .angularPosition(Rotations.of(motor.getPosition().getValueAsDouble()))
                    .angularVelocity(RotationsPerSecond.of(motor.getVelocity().getValueAsDouble())),
            this));
  }

  public Command intakeRollerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeRollerSysIdRoutine.quasistatic(direction).finallyDo(interrupted -> intakeStop());
  }

  public Command intakeRollerSysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeRollerSysIdRoutine.dynamic(direction).finallyDo(interrupted -> intakeStop());
  }

  public Command intakeArmSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return intakeArmSysIdRoutine
        .quasistatic(direction)
        .until(() -> isIntakeArmNearSysIdLimit(direction))
        .finallyDo(interrupted -> intakeArmMotor.set(0.0));
  }

  public Command intakeArmSysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeArmSysIdRoutine
        .dynamic(direction)
        .until(() -> isIntakeArmNearSysIdLimit(direction))
        .finallyDo(interrupted -> intakeArmMotor.set(0.0));
  }

  private boolean isIntakeArmNearSysIdLimit(SysIdRoutine.Direction direction) {
    double extensionMeters = getIntakeExtensionMeters();
    double minSafeExtension =
        IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS + INTAKE_ARM_SYSID_LIMIT_MARGIN_METERS;
    double maxSafeExtension =
        IntakeConstants.INTAKE_EXTENSION_MAX_METERS - INTAKE_ARM_SYSID_LIMIT_MARGIN_METERS;

    return direction == SysIdRoutine.Direction.kForward
        ? extensionMeters >= maxSafeExtension
        : extensionMeters <= minSafeExtension;
  }

  // --- Roller control ---

  public void intakeStop() {
    intakeMotor.set(0.0);
  }

  public void setIntakeSpeed(double velocity) {
    // Convert mechanism rps to motor-side rps through gear reduction.
    intakeMotor.setControl(
        intakeVelocityControl.withVelocity(velocity * IntakeConstants.INTAKE_GEAR_REDUCTION));
  }

  // --- Linear extension control ---

  public void setIntakeExtensionMeters(double extensionMeters) {
    if (!isIntakeHomed()) {
      return;
    }

    // Clamp requested extension to physical limits before commanding closed-loop position.
    double clampedExtension =
        Math.max(
            IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS,
            Math.min(extensionMeters, IntakeConstants.INTAKE_EXTENSION_MAX_METERS));

    double motorRotations = IntakeConstants.extensionMetersToMotorRotations(clampedExtension);
    intakeArmMotor.setControl(intakeArmPositionControl.withPosition(motorRotations));
  }

  public double getIntakeExtensionMeters() {
    double motorRotations = intakeArmPositionSignal.getValueAsDouble();
    return IntakeConstants.motorRotationsToExtensionMeters(motorRotations);
  }

  public boolean isIntakeDeployed() {
    return getIntakeExtensionMeters() >= INTAKE_DEPLOYED_THRESHOLD_METERS;
  }

  public boolean isIntakeRetracted() {
    return getIntakeExtensionMeters() <= INTAKE_RETRACTED_THRESHOLD_METERS;
  }

  public boolean isIntakeHomed() {
    return intakeHomed;
  }

  /** Re-seed extension sensor as fully retracted; call only when physically retracted. */
  public void homeIntakeAtRetractedPosition() {
    intakeArmMotor.setPosition(
        IntakeConstants.extensionMetersToMotorRotations(
            IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS));
    intakeHomed = true;
  }

  public void publishTelemetry() {
    double extensionMeters = getIntakeExtensionMeters();

    SmartDashboard.putNumber(
        "Intake/RollerVelocityRps",
        TelemetryConstants.roundTelemetry(intakeVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/RollerCurrentA",
        TelemetryConstants.roundTelemetry(intakeCurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/RollerVoltageV",
        TelemetryConstants.roundTelemetry(intakeVoltageSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/Roller2VelocityRps",
        TelemetryConstants.roundTelemetry(intake2VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/Roller2CurrentA",
        TelemetryConstants.roundTelemetry(intake2CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/Roller2VoltageV",
        TelemetryConstants.roundTelemetry(intake2VoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber(
        "Intake/ExtensionMotorPositionRot",
        TelemetryConstants.roundTelemetry(intakeArmPositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/ExtensionMotorVelocityRps",
        TelemetryConstants.roundTelemetry(intakeArmVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/ExtensionMotorCurrentA",
        TelemetryConstants.roundTelemetry(intakeArmCurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Intake/ExtensionMotorVoltageV",
        TelemetryConstants.roundTelemetry(intakeArmVoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber(
        "Intake/ExtensionAmountMeters", TelemetryConstants.roundTelemetry(extensionMeters));
    SmartDashboard.putNumber(
        "Intake/ExtensionAmountMillimeters",
        TelemetryConstants.roundTelemetry(extensionMeters * 1000.0));
    SmartDashboard.putBoolean("Intake/Homed", intakeHomed);
    if (isSimulationInitialized && intakeArmSim != null) {
      SmartDashboard.putNumber(
          "Intake/SimExtensionMeters",
          TelemetryConstants.roundTelemetry(intakeArmSim.getPositionMeters()));
    }
  }

  private void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        intakeVelocitySignal,
        // intakeCurrentSignal,
        // intakeVoltageSignal,
        intake2VelocitySignal,
        // intake2CurrentSignal,
        // intake2VoltageSignal,
        intakeArmPositionSignal
        // ,
        // intakeArmVelocitySignal,
        // intakeArmCurrentSignal,
        // intakeArmVoltageSignal
        );
  }

  // --- SIMULATION ---

  private DCMotor intakeDcMotor;
  private LinearSystem<N2, N1, N2> intakeSystem;
  private DCMotorSim intakeSim;

  private DCMotor intakeArmDcMotor;
  private ElevatorSim intakeArmSim;
  private TalonFXSimState intakeMotorSimState;
  private TalonFXSimState intakeMotor2SimState;
  private TalonFXSimState intakeArmMotorSimState;
  private double extensionMeters;
  private double extensionMetersPerSecond;
  private double armMotorRotations;
  private double armMotorRps;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {
    if (!isSimulationInitialized) {
      isSimulationInitialized = true;

      // Initialize roller simulation.
      intakeDcMotor = DCMotor.getKrakenX60(1);
      intakeSystem =
          LinearSystemId.createDCMotorSystem(
              intakeDcMotor,
              IntakeConstants.INTAKE_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
              IntakeConstants.INTAKE_GEAR_REDUCTION);
      intakeSim = new DCMotorSim(intakeSystem, intakeDcMotor);
      intakeMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      intakeMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
      intakeMotor2.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      intakeMotor2.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

      // Initialize extension-arm simulation.
      intakeArmDcMotor = DCMotor.getKrakenX60(1);
      intakeArmSim =
          new ElevatorSim(
              intakeArmDcMotor,
              IntakeConstants.INTAKE_ARM_GEAR_REDUCTION,
              IntakeConstants.INTAKE_ARM_MASS.in(edu.wpi.first.units.Units.Kilograms),
              IntakeConstants.INTAKE_SIM_DRUM_RADIUS_METERS,
              IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters),
              IntakeConstants.INTAKE_EXTENSION_MAX.in(Meters),
              false,
              IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters),
              0.0,
              0.0);
      intakeArmMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      intakeArmMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    } else {
      intakeMotorSimState = intakeMotor.getSimState();
      intakeMotor2SimState = intakeMotor2.getSimState();
      intakeArmMotorSimState = intakeArmMotor.getSimState();

      intakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      intakeMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      intakeArmMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      // Step roller simulation and write back rotor position/velocity.
      intakeSim.setInputVoltage(intakeMotorSimState.getMotorVoltage());
      intakeSim.update(0.002);

      intakeMotorSimState.setRawRotorPosition(
          intakeSim.getAngularPosition().times(IntakeConstants.INTAKE_GEAR_REDUCTION));
      intakeMotorSimState.setRotorVelocity(
          intakeSim.getAngularVelocity().times(IntakeConstants.INTAKE_GEAR_REDUCTION));

      intakeMotor2SimState.setRawRotorPosition(
          intakeSim.getAngularPosition().times(IntakeConstants.INTAKE_GEAR_REDUCTION));
      intakeMotor2SimState.setRotorVelocity(
          intakeSim.getAngularVelocity().times(IntakeConstants.INTAKE_GEAR_REDUCTION));

      // Step extension simulation and write back arm rotor state.
      intakeArmSim.setInput(intakeArmMotorSimState.getMotorVoltage());
      intakeArmSim.update(0.002);

      extensionMeters = intakeArmSim.getPositionMeters();
      extensionMetersPerSecond = intakeArmSim.getVelocityMetersPerSecond();

      armMotorRotations = IntakeConstants.extensionMetersToMotorRotations(extensionMeters);
      armMotorRps = IntakeConstants.extensionMetersToMotorRotations(extensionMetersPerSecond);

      intakeArmMotorSimState.setRawRotorPosition(
          edu.wpi.first.units.Units.Rotations.of(armMotorRotations));
      intakeArmMotorSimState.setRotorVelocity(RotationsPerSecond.of(armMotorRps));
    }
  }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // --- End of simulation ---

  public void close() {
    // Fully retract and stop rollers.
    intakeGoalVelocity = 0;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_RETRACTED_METERS;
    intakeStop();
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
  }

  public void deploy() {
    // Deploy arm without running rollers.
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED_METERS;
    intakeGoalVelocity = 0;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    intakeStop();
  }

  public void intake() {
    // Deploy then run intake rollers at acquisition speed.
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED_METERS;
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY_RPS;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void intakeWithOffset() {
    // Alternative intake mode currently mapped to deployed intake behavior.
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY_RPS;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED_METERS;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void feed() {
    // Move to feed position and run rollers toward feeder path.
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY_RPS;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_FEED_METERS;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void reverse() {
    // Reverse rollers; only run once intake is deployed to avoid stalling retracted geometry.
    intakeGoalVelocity = IntakeConstants.INTAKE_REVERSE_VELOCITY_RPS;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED_METERS;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    if (isIntakeDeployed()) {
      setIntakeSpeed(intakeGoalVelocity);
    }
  }

  public void idleBetween() {
    // Intermediate hold state used between intake and shooter handoff.
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY_RPS;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_FEED_METERS;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  /**
   * Drive intake arm in reverse until it stalls on the retracted hardstop, then reseed encoder to
   * retracted zero.
   *
   * <p>Starts the non-blocking process. Call {@link #updateIntakeHardstopZeroing()} repeatedly
   * (e.g., from a command execute) until complete.
   */
  public void zeroIntakeAtHardstop() {
    startIntakeHardstopZeroing();
  }

  public void startIntakeHardstopZeroing() {
    if (intakeHardstopZeroingActive) {
      return;
    }

    intakeHomed = false;
    intakeHardstopZeroingActive = true;
    intakeHardstopZeroingComplete = false;
    intakeHardstopZeroingSucceeded = false;
    intakeHardstopZeroingStartSec = Timer.getFPGATimestamp();
    intakeHardstopStallStartSec = -1.0;

    // Stop rollers while zeroing extension.
    intakeGoalVelocity = 0.0;
    intakeStop();
  }

  public void updateIntakeHardstopZeroing() {
    if (!intakeHardstopZeroingActive || intakeHardstopZeroingComplete) {
      return;
    }

    refreshStatusSignals();

    double nowSec = Timer.getFPGATimestamp();
    double elapsedSec = nowSec - intakeHardstopZeroingStartSec;
    double armVelAbsRps;

    // Keep retracting toward hardstop.
    intakeArmMotor.set(INTAKE_ARM_ZEROING_REVERSE_OUTPUT);
    armVelAbsRps = Math.abs(intakeArmVelocitySignal.getValueAsDouble());

    if (armVelAbsRps <= INTAKE_ARM_ZEROING_STALL_VELOCITY_RPS) {
      if (intakeHardstopStallStartSec < 0.0) {
        intakeHardstopStallStartSec = nowSec;
      }

      if ((nowSec - intakeHardstopStallStartSec) >= INTAKE_ARM_ZEROING_STALL_DEBOUNCE_SEC) {
        intakeArmMotor.set(0.0);
        homeIntakeAtRetractedPosition();
        intakeHardstopZeroingActive = false;
        intakeHardstopZeroingComplete = true;
        intakeHardstopZeroingSucceeded = true;
      }
    } else {
      intakeHardstopStallStartSec = -1.0;
    }

    if (elapsedSec >= INTAKE_ARM_ZEROING_TIMEOUT_SEC) {
      intakeArmMotor.set(0.0);
      intakeHardstopZeroingActive = false;
      intakeHardstopZeroingComplete = true;
      intakeHardstopZeroingSucceeded = false;
      intakeHardstopStallStartSec = -1.0;
    }
  }

  public void stopIntakeHardstopZeroing() {
    intakeArmMotor.set(0.0);
    intakeHardstopZeroingActive = false;
  }

  public boolean isIntakeHardstopZeroingComplete() {
    return intakeHardstopZeroingComplete;
  }

  public boolean didIntakeHardstopZeroingSucceed() {
    return intakeHardstopZeroingSucceeded;
  }

  public boolean isIntakeHardstopZeroingActive() {
    return intakeHardstopZeroingActive;
  }

  public void test() {
    intakeGoalVelocity = intakeTestRPM;
    intakeGoalExtensionMeters = intakeTestExtensionMeters;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  /** Set intake test speed/extension, then apply immediately. */
  public void test(double intakeRps, double intakeExtensionMeters) {
    intakeTestRPM = intakeRps;
    intakeTestExtensionMeters = intakeExtensionMeters;
    test();
  }

  @Override
  public void periodic() {
    refreshStatusSignals();
  }
}
