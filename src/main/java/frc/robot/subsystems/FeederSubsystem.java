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

  private double feederBeltGoalVelocity;
  private double feederFeedGoalVelocity;
  private double feederFeedTestRPM;
  private boolean isFeederReady = false;
  private int feederReadyLoops = 0;
  private static final int FEEDER_READY_REQUIRED_LOOPS = 3;

  private static final int FEEDER_FEED_VELOCITY_AVG_SAMPLES = 5;
  private final double[] feederFeedVelocityWindow = new double[FEEDER_FEED_VELOCITY_AVG_SAMPLES];
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
    if (shooterGoalRpsSupplier == null) {
      return DEFAULT_FEEDER_FEED_RPS;
    }
    return Math.abs(shooterGoalRpsSupplier.get());
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
    feederFeedMotor.set(0.0);
  }

  public void feederBeltSetSpeed(double velocity) {
    // Convert mechanism rps to motor-side rps through gear reduction.
    feederBeltMotor.setControl(
        feederVelocityControl.withVelocity(velocity * FeederConstants.FEEDER_BELT_GEAR_REDUCTION));
  }

  public void feederFeedSetSpeed(double velocity) {
    // Feed motor is configured as a follower of belt motor; command master speed only.
    feederBeltSetSpeed(velocity);
  }

  public double getFeederFeedVelocity() {
    return feederFeedVelocityAverageRps;
  }

  public boolean isFeederReady() {
    return isFeederReady;
  }

  /** Get the current feeder belt goal velocity in RPS. */
  public double getFeederBeltGoalVelocityRps() {
    return feederBeltGoalVelocity;
  }

  /** Get the current feeder feed wheel goal velocity in RPS. */
  public double getFeederFeedGoalVelocityRps() {
    return feederFeedGoalVelocity;
  }

  double newSample;

  private void updateFeederFeedVelocityAverage() {
    newSample = feederFeedVelocitySignal.getValueAsDouble() / FeederConstants.FEEDER_GEAR_REDUCTION;

    if (feederFeedVelocityWindowCount < FEEDER_FEED_VELOCITY_AVG_SAMPLES) {
      feederFeedVelocityWindowCount++;
    } else {
      feederFeedVelocityWindowSum -= feederFeedVelocityWindow[feederFeedVelocityWindowIndex];
    }

    feederFeedVelocityWindow[feederFeedVelocityWindowIndex] = newSample;
    feederFeedVelocityWindowSum += newSample;
    feederFeedVelocityWindowIndex =
        (feederFeedVelocityWindowIndex + 1) % FEEDER_FEED_VELOCITY_AVG_SAMPLES;

    feederFeedVelocityAverageRps = feederFeedVelocityWindowSum / feederFeedVelocityWindowCount;
  }

  public boolean isFeederFeedAtGoalSpeed() {
    return Math.abs(getFeederFeedVelocity() - feederFeedGoalVelocity)
        < FeederConstants.FEEDER_ALLOWABLE_ERROR_RPS;
  }

  private void updateFeederReadyLatch() {
    if (Math.abs(feederFeedGoalVelocity) <= 0.0) {
      feederReadyLoops = 0;
      isFeederReady = false;
      return;
    }

    if (getFeederFeedVelocity() > (Math.abs(feederFeedGoalVelocity) * 0.5)) {
      feederReadyLoops++;
    } else {
      feederReadyLoops = 0;
    }

    isFeederReady = feederReadyLoops >= FEEDER_READY_REQUIRED_LOOPS;
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
        "Feeder/BeltMotorGoalVelocityRps",
        TelemetryConstants.roundTelemetry(feederBeltGoalVelocity));
    SmartDashboard.putNumber(
        "Feeder/BeltMotorVelocityErrorRps",
        TelemetryConstants.roundTelemetry(
            feederBeltGoalVelocity
                - (feederBeltVelocitySignal.getValueAsDouble()
                    / FeederConstants.FEEDER_BELT_GEAR_REDUCTION)));
    SmartDashboard.putNumber(
        "feederBeltRpsError",
        TelemetryConstants.roundTelemetry(
            feederBeltGoalVelocity
                - (feederBeltVelocitySignal.getValueAsDouble()
                    / FeederConstants.FEEDER_BELT_GEAR_REDUCTION)));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorGoalVelocityRps",
        TelemetryConstants.roundTelemetry(feederFeedGoalVelocity));
    SmartDashboard.putNumber(
        "Feeder/FeedMotorVelocityErrorRps",
        TelemetryConstants.roundTelemetry(feederFeedGoalVelocity - getFeederFeedVelocity()));
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
    feederBeltGoalVelocity = 0;
    feederFeedGoalVelocity = 0;
    feederStop();
  }

  public void feed() {
    // Single-master follower setup: belt/feed move together at one commanded speed.
    feederFeedGoalVelocity = getShooterGoalRpsForFeed();
    feederBeltGoalVelocity = feederFeedGoalVelocity;

    feederBeltSetSpeed(feederBeltGoalVelocity);
  }

  public void feedGetReady() {
    // In follower setup, get-ready uses the same commanded velocity path as feed.
    feederFeedGoalVelocity = getShooterGoalRpsForFeed();
    feederBeltGoalVelocity = feederFeedGoalVelocity;
    feederBeltSetSpeed(feederBeltGoalVelocity);
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
    feederBeltGoalVelocity = FeederConstants.FEEDER_REVERSE_VELOCITY_RPS;
    feederFeedGoalVelocity = FeederConstants.FEEDER_REVERSE_VELOCITY_RPS;
    feederBeltSetSpeed(feederBeltGoalVelocity);
  }

  public void test() {
    // In follower mode, use one velocity command for both motors.
    feederBeltGoalVelocity = feederFeedTestRPM;
    feederFeedGoalVelocity = feederFeedTestRPM;
    feederBeltSetSpeed(feederBeltGoalVelocity);
  }

  /** Set independent test RPS values for belt and feed motors, then apply immediately. */
  public void test(double feederBeltRps, double feederFeedRps) {
    feederFeedTestRPM = feederFeedRps;
    test();
  }

  /**
   * @deprecated Use {@link #test(double, double)}.
   */
  @Deprecated
  public void test(double feederRps) {
    test(feederRps, feederRps);
  }

  @Override
  public void periodic() {
    refreshStatusSignals();
    updateFeederFeedVelocityAverage();
    updateFeederReadyLatch();
  }
}
