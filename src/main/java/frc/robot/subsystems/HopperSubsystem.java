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
import frc.robot.constants.HopperConstants;
import frc.robot.constants.TelemetryConstants;

/**
 * Controls hopper transport motors that stage cargo between intake and feeder.
 */
public class HopperSubsystem extends SubsystemBase {

  private TalonFX hopperMotor;
  private TalonFX hopperMotor2;
  private TalonFX hopperSideMotor;


  private final VelocityVoltage hopperVelocityControl;
  private final StatusSignal<?> hopper1PositionSignal;
  private final StatusSignal<?> hopper1VelocitySignal;
  private final StatusSignal<?> hopper1CurrentSignal;
  private final StatusSignal<?> hopper1VoltageSignal;
  private final StatusSignal<?> hopper2PositionSignal;
  private final StatusSignal<?> hopper2VelocitySignal;
  private final StatusSignal<?> hopper2CurrentSignal;
  private final StatusSignal<?> hopper2VoltageSignal;
  private final StatusSignal<?> hopper3PositionSignal;
  private final StatusSignal<?> hopper3VelocitySignal;
  private final StatusSignal<?> hopper3CurrentSignal;
  private final StatusSignal<?> hopper3VoltageSignal;

  private double hopperGoalVelocity;
  private double hopperTestRPM;

  private final VoltageOut hopperMainSysIdControl;
  private final VoltageOut hopperSideSysIdControl;
  private final SysIdRoutine hopperMainSysIdRoutine;
  private final SysIdRoutine hopperSideSysIdRoutine;

  /** Creates a new hopperSubsystem. */
  public HopperSubsystem() {
    hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);
    hopperMotor2 = new TalonFX(HopperConstants.HOPPER_MOTOR_2_ID);
  hopperSideMotor = new TalonFX(HopperConstants.HOPPER_SIDE_MOTOR_ID);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hopperMotor.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = hopperMotor2.getConfigurator().apply(HopperConstants.HOPPER_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
  status = hopperSideMotor.getConfigurator().apply(HopperConstants.HOPPER_SIDE_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    hopper1PositionSignal = hopperMotor.getPosition(false);
    hopper1VelocitySignal = hopperMotor.getVelocity(false);
    hopper1CurrentSignal = hopperMotor.getStatorCurrent(false);
    hopper1VoltageSignal = hopperMotor.getMotorVoltage(false);

    hopper2PositionSignal = hopperMotor2.getPosition(false);
    hopper2VelocitySignal = hopperMotor2.getVelocity(false);
    hopper2CurrentSignal = hopperMotor2.getStatorCurrent(false);
    hopper2VoltageSignal = hopperMotor2.getMotorVoltage(false);

  hopper3PositionSignal = hopperSideMotor.getPosition(false);
  hopper3VelocitySignal = hopperSideMotor.getVelocity(false);
  hopper3CurrentSignal = hopperSideMotor.getStatorCurrent(false);
  hopper3VoltageSignal = hopperSideMotor.getMotorVoltage(false);

    hopperMainSysIdControl = new VoltageOut(0).withEnableFOC(false);
    hopperSideSysIdControl = new VoltageOut(0).withEnableFOC(false);
  hopperMainSysIdRoutine = createSysIdRoutine("hopper/Main", hopperMotor, hopperMainSysIdControl);
  hopperSideSysIdRoutine = createSysIdRoutine("hopper/Side", hopperSideMotor, hopperSideSysIdControl);
    
    hopperVelocityControl = HopperConstants.HOPPER_VELOCITY_CONTROL.clone();
  }

  private SysIdRoutine createSysIdRoutine(String logPrefix, TalonFX motor, VoltageOut voltageControl) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            state -> SignalLogger.writeString(logPrefix + "_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
              motor.setControl(voltageControl.withOutput(output.in(Volts)));
              SignalLogger.writeDouble(logPrefix + "_Voltage", motor.getMotorVoltage().getValueAsDouble());
              SignalLogger.writeDouble(logPrefix + "_Position", motor.getPosition().getValueAsDouble());
              SignalLogger.writeDouble(logPrefix + "_Velocity", motor.getVelocity().getValueAsDouble());
            },
            null,
            this
        )
    );
  }

  public Command hopperMainSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return hopperMainSysIdRoutine.quasistatic(direction).finallyDo(interrupted -> hopperStop());
  }

  public Command hopperMainSysIdDynamic(SysIdRoutine.Direction direction) {
    return hopperMainSysIdRoutine.dynamic(direction).finallyDo(interrupted -> hopperStop());
  }

  public Command hopperSideSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return hopperSideSysIdRoutine.quasistatic(direction).finallyDo(interrupted -> {
      hopperSideMotor.set(0.0);
    });
  }

  public Command hopperSideSysIdDynamic(SysIdRoutine.Direction direction) {
    return hopperSideSysIdRoutine.dynamic(direction).finallyDo(interrupted -> {
      hopperSideMotor.set(0.0);
    });
  }

  public void hopperStop() {
    hopperMotor.set(0.0);
    hopperSideMotor.set(0.0);
  }

  public void setHopperSpeed(double velocity) {
    // Convert mechanism rps to motor-side rps through gear reduction.
    setHopperSpeeds(velocity, velocity);
  }

  public void setHopperSpeeds(double vel1, double vel2)
  {
        // Convert mechanism rps to motor-side rps through gear reduction.
    hopperMotor.setControl(
      hopperVelocityControl.withVelocity(vel1 * HopperConstants.HOPPER_GEAR_REDUCTION)
    );
    hopperSideMotor.setControl(
      hopperVelocityControl.withVelocity(vel2 * HopperConstants.HOPPER_SIDE_GEAR_REDUCTION)
    );
  }

  /** @deprecated Use {@link #setHopperSpeed(double)}. */
  @Deprecated
  public void sethopperSpeed(double velocity) {
    setHopperSpeed(velocity);
  }

  public void publishTelemetry() {
    SmartDashboard.putNumber("Hopper/Motor1PositionRot", TelemetryConstants.roundTelemetry(hopper1PositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor1VelocityRps", TelemetryConstants.roundTelemetry(hopper1VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor1CurrentA", TelemetryConstants.roundTelemetry(hopper1CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor1VoltageV", TelemetryConstants.roundTelemetry(hopper1VoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Hopper/Motor2PositionRot", TelemetryConstants.roundTelemetry(hopper2PositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor2VelocityRps", TelemetryConstants.roundTelemetry(hopper2VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor2CurrentA", TelemetryConstants.roundTelemetry(hopper2CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor2VoltageV", TelemetryConstants.roundTelemetry(hopper2VoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Hopper/Motor3PositionRot", TelemetryConstants.roundTelemetry(hopper3PositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor3VelocityRps", TelemetryConstants.roundTelemetry(hopper3VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor3CurrentA", TelemetryConstants.roundTelemetry(hopper3CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Hopper/Motor3VoltageV", TelemetryConstants.roundTelemetry(hopper3VoltageSignal.getValueAsDouble()));
  }

  private void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        //hopper1PositionSignal,
        hopper1VelocitySignal,
        //hopper1CurrentSignal,
        //hopper1VoltageSignal,
        //hopper2PositionSignal,
        hopper2VelocitySignal,
        //hopper2CurrentSignal,
        //hopper2VoltageSignal,
        //hopper3PositionSignal,
        hopper3VelocitySignal
        //hopper3CurrentSignal,
        //hopper3VoltageSignal
        );
  }

  // SIMULATION

  private DCMotor hopperDcMotor;
  private LinearSystem<N2, N1, N2> hopperSystem;
  private DCMotorSim hopperSim;
  private TalonFXSimState hopperMotorSimState;
  private TalonFXSimState hopperMotor2SimState;
  private TalonFXSimState hopperMotor3SimState;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {

        if (!isSimulationInitialized) {
      // Initialize hopper simulation model once.
            isSimulationInitialized = true;
            hopperDcMotor = DCMotor.getKrakenX60(1);
            hopperSystem = LinearSystemId.createDCMotorSystem(
            hopperDcMotor,
            HopperConstants.HOPPER_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
            HopperConstants.HOPPER_GEAR_REDUCTION);

            hopperSim = new DCMotorSim(hopperSystem, hopperDcMotor);
            hopperMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hopperMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            hopperMotor2.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            hopperMotor2.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            hopperSideMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            hopperSideMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
      // Step simulation and mirror state into both hopper motors.
            hopperMotorSimState = hopperMotor.getSimState();
            hopperMotor2SimState = hopperMotor2.getSimState();
            hopperMotor3SimState = hopperSideMotor.getSimState();

            hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            hopperMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            hopperMotor3SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            
            hopperSim.setInputVoltage(hopperMotorSimState.getMotorVoltage());
            hopperSim.update(0.002);

            hopperMotorSimState.setRawRotorPosition(
                hopperSim.getAngularPosition().times(HopperConstants.HOPPER_GEAR_REDUCTION));
            hopperMotorSimState.setRotorVelocity(
                hopperSim.getAngularVelocity().times(HopperConstants.HOPPER_GEAR_REDUCTION));

            hopperMotor2SimState.setRawRotorPosition(
                hopperSim.getAngularPosition().times(HopperConstants.HOPPER_GEAR_REDUCTION));
            hopperMotor2SimState.setRotorVelocity(
                hopperSim.getAngularVelocity().times(HopperConstants.HOPPER_GEAR_REDUCTION));

      hopperMotor3SimState.setRawRotorPosition(
        hopperSim.getAngularPosition().times(HopperConstants.HOPPER_GEAR_REDUCTION));
      hopperMotor3SimState.setRotorVelocity(
        hopperSim.getAngularVelocity().times(HopperConstants.HOPPER_GEAR_REDUCTION));
        }   
    }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // End of simulation part

  public void zero() {
    // Stop hopper flow.
    hopperGoalVelocity = 0;
    hopperStop();
  }

  public void feed() {
    // Normal forward transport toward feeder.
    hopperGoalVelocity = HopperConstants.HOPPER_FEEDING_VELOCITY_RPS;
    setHopperSpeed(hopperGoalVelocity);
  }

  public void push() {
    // Faster forward push mode for quick transfer.
    hopperGoalVelocity = 0;
    setHopperSpeeds(hopperGoalVelocity, HopperConstants.HOPPER_SIDE_PUSHING_VELOCITY_RPS);
  }

  public void reverse() {
    // Reverse direction to clear jams.
    hopperGoalVelocity = HopperConstants.HOPPER_REVERSE_VELOCITY_RPS;
    setHopperSpeed(hopperGoalVelocity);
  }

  public void test() {
    hopperGoalVelocity = hopperTestRPM;
    setHopperSpeed(hopperGoalVelocity);
  }

  /** Set hopper test RPS, then apply immediately. */
  public void test(double hopperRps) {
    hopperTestRPM = hopperRps;
    test();
  }


  @Override
  public void periodic() {
    refreshStatusSignals();
  }
}
