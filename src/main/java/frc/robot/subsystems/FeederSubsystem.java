// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
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
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.FeederConstants;
import frc.robot.constants.TelemetryConstants;
import java.util.function.Supplier;

/** Controls belt + feed motors that transfer cargo from hopper path into shooter. */
public class FeederSubsystem extends SubsystemBase {

  private TalonFX feederBeltMotor;
  private TalonFX feederFeedMotor;

  private final VelocityVoltage feederVelocityControl;
  private final StatusSignal<?> feederBeltPositionSignal;
  private final StatusSignal<?> feederBeltVelocitySignal;
  private final StatusSignal<?> feederBeltVoltageSignal;
  private final StatusSignal<?> feederFeedPositionSignal;
  private final StatusSignal<?> feederFeedVelocitySignal;
  private final StatusSignal<?> feederFeedVoltageSignal;
  private final Supplier<Double> shooterGoalRpsSupplier;

  private double feederGoalVelocity;
  private double feederTestRPS;
  private boolean isFeederReady = false;
  private int feederReadyLoops = 0;
  private final double[] feederFeedVelocityWindow =
      new double[FeederConstants.FEEDER_VELOCITY_AVG_SAMPLES];
  private int feederFeedVelocityWindowIndex = 0;
  private int feederFeedVelocityWindowCount = 0;
  private double feederFeedVelocityWindowSum = 0.0;
  private double feederFeedVelocityAverageRps = 0.0;

  private final VoltageOut feederBeltSysIdControl;
  private final VoltageOut feederFeedSysIdControl;
  private final SysIdRoutine feederBeltSysIdRoutine;
  private final SysIdRoutine feederFeedSysIdRoutine;

  private static final double DEFAULT_FEEDER_FEED_RPS = FeederConstants.FEEDER_FEEDING_VELOCITY_RPS;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    this(() -> DEFAULT_FEEDER_FEED_RPS);
  }

  /** Creates a new FeederSubsystem with shooter goal RPS supplier for feed target coupling. */
  public FeederSubsystem(Supplier<Double> shooterGoalRpsSupplier) {
    feederBeltMotor = new TalonFX(FeederConstants.FEEDER_BELT_MOTOR_ID);
    feederFeedMotor = new TalonFX(FeederConstants.FEEDER_FEED_MOTOR_ID);
    this.shooterGoalRpsSupplier = shooterGoalRpsSupplier;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = feederBeltMotor.getConfigurator().apply(FeederConstants.FEEDER_BELT_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = feederFeedMotor.getConfigurator().apply(FeederConstants.FEEDER_FEED_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println(
          "Could not apply configs to feeder motor 2, error code: " + status.toString());
    }

    // Mechanical update: feed motor is now coupled with belt motor and follows same orientation.
    feederFeedMotor.setControl(
        new Follower(feederBeltMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    feederBeltPositionSignal = feederBeltMotor.getPosition(false);
    feederBeltVelocitySignal = feederBeltMotor.getVelocity(false);
    feederBeltVoltageSignal = feederBeltMotor.getMotorVoltage(false);
    feederFeedPositionSignal = feederFeedMotor.getPosition(false);
    feederFeedVelocitySignal = feederFeedMotor.getVelocity(false);
    feederFeedVoltageSignal = feederFeedMotor.getMotorVoltage(false);

    feederBeltSysIdControl = new VoltageOut(0).withEnableFOC(false);
    feederFeedSysIdControl = new VoltageOut(0).withEnableFOC(false);
    feederBeltSysIdRoutine =
        createSysIdRoutine("feeder/Belt", feederBeltMotor, feederBeltSysIdControl);
    feederFeedSysIdRoutine =
        createSysIdRoutine("feeder/Feed", feederFeedMotor, feederFeedSysIdControl);

    feederVelocityControl = FeederConstants.FEEDER_VELOCITY_CONTROL.clone();
  }

  private double getShooterGoalRpsForFeed() {
    return DEFAULT_FEEDER_FEED_RPS;
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
            null,
            this));
  }

  public Command feederBeltSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return feederBeltSysIdRoutine
        .quasistatic(direction)
        .finallyDo(interrupted -> feederBeltMotor.set(0.0));
  }

  public Command feederBeltSysIdDynamic(SysIdRoutine.Direction direction) {
    return feederBeltSysIdRoutine
        .dynamic(direction)
        .finallyDo(interrupted -> feederBeltMotor.set(0.0));
  }

  public Command feederFeedSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return feederFeedSysIdRoutine
        .quasistatic(direction)
        .finallyDo(interrupted -> feederFeedMotor.set(0.0));
  }

  public Command feederFeedSysIdDynamic(SysIdRoutine.Direction direction) {
    return feederFeedSysIdRoutine
        .dynamic(direction)
        .finallyDo(interrupted -> feederFeedMotor.set(0.0));
  }

  public void feederStop() {
    isFeederReady = false;
    feederReadyLoops = 0;
    feederBeltMotor.set(0.0);
  }

  public void feederSetSpeed(double velocity) {
    // Convert mechanism rps to motor-side rps through gear reduction.
    feederBeltMotor.setControl(
        feederVelocityControl.withVelocity(velocity * FeederConstants.FEEDER_BELT_GEAR_REDUCTION));
  }

  public void feederFeedSetSpeed(double velocity) {
    // Feed motor is configured as a follower of belt motor; command master speed only.
    feederSetSpeed(velocity);
  }

  public double getFeederFeedVelocity() {
    return feederFeedVelocityAverageRps;
  }

  public boolean isFeederReady() {
    return isFeederReady;
  }

  /** Get the current feeder goal velocity in RPS. */
  public double getFeederGoalVelocityRps() {
    return feederGoalVelocity;
  }

  double newSample;

  private void updateFeederFeedVelocityAverage() {
    newSample = feederFeedVelocitySignal.getValueAsDouble() / FeederConstants.FEEDER_GEAR_REDUCTION;

    if (feederFeedVelocityWindowCount < FeederConstants.FEEDER_VELOCITY_AVG_SAMPLES) {
      feederFeedVelocityWindowCount++;
    } else {
      feederFeedVelocityWindowSum -= feederFeedVelocityWindow[feederFeedVelocityWindowIndex];
    }

    feederFeedVelocityWindow[feederFeedVelocityWindowIndex] = newSample;
    feederFeedVelocityWindowSum += newSample;
    feederFeedVelocityWindowIndex =
        (feederFeedVelocityWindowIndex + 1) % FeederConstants.FEEDER_VELOCITY_AVG_SAMPLES;

    feederFeedVelocityAverageRps = feederFeedVelocityWindowSum / feederFeedVelocityWindowCount;
  }

  public boolean isFeederFeedAtGoalSpeed() {
    return Math.abs(getFeederFeedVelocity() - feederGoalVelocity)
        < FeederConstants.FEEDER_ALLOWABLE_ERROR_RPS;
  }

  private void updateFeederReadyLatch() {
    if (Math.abs(feederGoalVelocity) <= 0.0) {
      feederReadyLoops = 0;
      isFeederReady = false;
      return;
    }

    if (getFeederFeedVelocity() > (Math.abs(feederGoalVelocity) * 0.5)) {
      feederReadyLoops++;
    } else {
      feederReadyLoops = 0;
    }

    isFeederReady = feederReadyLoops >= FeederConstants.FEEDER_READY_REQUIRED_LOOPS;
  }

  // ===== DATA LOGGING =====

  private final DoubleLogEntry logBeltVelocityRps  = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/BeltVelocityRps");
  private final DoubleLogEntry logGoalRps          = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/GoalRps");
  private final DoubleLogEntry logBeltErrorRps     = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/BeltErrorRps");
  private final DoubleLogEntry logBeltCurrentA     = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/BeltCurrentA");
  private final DoubleLogEntry logFeedVelocityRps  = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/FeedVelocityAvgRps");
  private final DoubleLogEntry logFeedErrorRps     = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/FeedErrorRps");
  private final BooleanLogEntry logIsReady         = new BooleanLogEntry(DataLogManager.getLog(), "/Log/Feeder/IsReady");
  private final DoubleLogEntry logReadyLoops       = new DoubleLogEntry(DataLogManager.getLog(), "/Log/Feeder/ReadyLoops");

  double beltActualRps;
  public void logData() {
    beltActualRps = feederBeltVelocitySignal.getValueAsDouble() / FeederConstants.FEEDER_BELT_GEAR_REDUCTION;
    logBeltVelocityRps.append(beltActualRps);
    logGoalRps.append(feederGoalVelocity);
    logBeltErrorRps.append(feederGoalVelocity - beltActualRps);
    logBeltCurrentA.append(feederBeltMotor.getSupplyCurrent().getValueAsDouble());
    logFeedVelocityRps.append(feederFeedVelocityAverageRps);
    logFeedErrorRps.append(feederGoalVelocity - feederFeedVelocityAverageRps);
    logIsReady.append(isFeederReady);
    logReadyLoops.append(feederReadyLoops);
  }

  public void publishTelemetry() {
    SmartDashboard.putNumber(
        "Feeder/BeltMotorPositionRot",
        TelemetryConstants.roundTelemetry(feederBeltPositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/BeltMotorVelocityRps",
        TelemetryConstants.roundTelemetry(feederBeltVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/BeltMotorCurrentA",
        TelemetryConstants.roundTelemetry(feederBeltMotor.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/BeltMotorVoltageV",
        TelemetryConstants.roundTelemetry(feederBeltVoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber(
        "Feeder/FeedMotorPositionRot",
        TelemetryConstants.roundTelemetry(feederFeedPositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorVelocityRps",
        TelemetryConstants.roundTelemetry(feederFeedVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorCurrentA",
        TelemetryConstants.roundTelemetry(feederFeedMotor.getSupplyCurrent().getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorVoltageV",
        TelemetryConstants.roundTelemetry(feederFeedVoltageSignal.getValueAsDouble()));
    SmartDashboard.putNumber(
        "Feeder/GoalVelocityRps",
        TelemetryConstants.roundTelemetry(feederGoalVelocity));
    SmartDashboard.putNumber(
        "Feeder/BeltMotorVelocityErrorRps",
        TelemetryConstants.roundTelemetry(
            feederGoalVelocity
                - (feederBeltVelocitySignal.getValueAsDouble()
                    / FeederConstants.FEEDER_BELT_GEAR_REDUCTION)));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorVelocityErrorRps",
        TelemetryConstants.roundTelemetry(feederGoalVelocity - getFeederFeedVelocity()));
    SmartDashboard.putBoolean("Feeder/FeedMotorAtGoalSpeed", isFeederFeedAtGoalSpeed());
    SmartDashboard.putBoolean("Feeder/IsFeederReady", isFeederReady);
    SmartDashboard.putNumber("Feeder/ReadyLoops", feederReadyLoops);
  }

  private void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        // feederBeltPositionSignal,
        feederBeltVelocitySignal,
        // feederBeltCurrentSignal,
        // feederBeltVoltageSignal,
        // feederFeedPositionSignal,
        feederFeedVelocitySignal
        // feederFeedCurrentSignal,
        // feederFeedVoltageSignal
        );
  }

  // SIMULATION

  private DCMotor feederDcMotor;
  private LinearSystem<N2, N1, N2> feederSystem;
  private DCMotorSim feederSim;
  private TalonFXSimState feederBeltMotorSimState;
  private TalonFXSimState feederFeedMotorSimState;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {

    if (!isSimulationInitialized) {
      // Initialize feeder simulation model once.
      isSimulationInitialized = true;
      feederDcMotor = DCMotor.getKrakenX60(1);
      feederSystem =
          LinearSystemId.createDCMotorSystem(
              feederDcMotor,
              FeederConstants.FEEDER_MOMENT_OF_INERTIA.in(
                  KilogramSquareMeters), // Moment of Inertia
              FeederConstants.FEEDER_BELT_GEAR_REDUCTION);

      feederSim = new DCMotorSim(feederSystem, feederDcMotor);
      feederBeltMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      feederBeltMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
      feederFeedMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      feederFeedMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    } else {
      // Step feeder simulation and mirror state to both motors.
      feederBeltMotorSimState = feederBeltMotor.getSimState();
      feederFeedMotorSimState = feederFeedMotor.getSimState();

      feederBeltMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      feederFeedMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      feederSim.setInputVoltage(feederBeltMotorSimState.getMotorVoltage());
      feederSim.update(0.002);

      feederBeltMotorSimState.setRawRotorPosition(
          feederSim.getAngularPosition().times(FeederConstants.FEEDER_BELT_GEAR_REDUCTION));

      feederBeltMotorSimState.setRotorVelocity(
          feederSim.getAngularVelocity().times(FeederConstants.FEEDER_BELT_GEAR_REDUCTION));

      feederFeedMotorSimState.setRawRotorPosition(
          feederSim.getAngularPosition().times(FeederConstants.FEEDER_GEAR_REDUCTION));

      feederFeedMotorSimState.setRotorVelocity(
          feederSim.getAngularVelocity().times(FeederConstants.FEEDER_GEAR_REDUCTION));
    }
  }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // End of simulation part

  public void zero() {
    isFeederReady = false;
    feederReadyLoops = 0;
    feederGoalVelocity = 0;
    feederStop();
  }

  public void feed() {
    feederGoalVelocity = getShooterGoalRpsForFeed();
    feederSetSpeed(feederGoalVelocity);
  }

  public void feedGetReady() {
    feederGoalVelocity = getShooterGoalRpsForFeed();
    feederSetSpeed(feederGoalVelocity);
  }

  /**
   * @deprecated Use {@link #feedGetReady()}.
   */
  @Deprecated
  public void feed_get_ready() {
    feedGetReady();
  }

  public void reverse() {
    // Reverse both motors to clear jams.
    isFeederReady = false;
    feederReadyLoops = 0;
    feederGoalVelocity = FeederConstants.FEEDER_REVERSE_VELOCITY_RPS;
    feederSetSpeed(feederGoalVelocity);
  }

  public void test() {
    // In follower mode, use one velocity command for both motors.
    feederGoalVelocity = feederTestRPS;
    feederSetSpeed(feederGoalVelocity);
  }

  public void test(double feederRps) {
    feederTestRPS = feederRps;
    test();
  }

  @Override
  public void periodic() {
    refreshStatusSignals();
    updateFeederFeedVelocityAverage();
    updateFeederReadyLatch();
  }
}
