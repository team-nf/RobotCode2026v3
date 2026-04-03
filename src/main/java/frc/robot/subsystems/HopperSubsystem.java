// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;

/**
 * Controls hopper transport motors that stage cargo between intake and feeder.
 */
public class HopperSubsystem extends SubsystemBase {

  private TalonFX hopperMotor;
  private TalonFX hopperMotor2;


  private final VelocityVoltage hopperVelocityControl;
  private final StatusSignal<?> hopper1PositionSignal;
  private final StatusSignal<?> hopper1VelocitySignal;
  private final StatusSignal<?> hopper1CurrentSignal;
  private final StatusSignal<?> hopper1VoltageSignal;
  private final StatusSignal<?> hopper2PositionSignal;
  private final StatusSignal<?> hopper2VelocitySignal;
  private final StatusSignal<?> hopper2CurrentSignal;
  private final StatusSignal<?> hopper2VoltageSignal;

  private double hopperGoalVelocity;
  private double hopperTestRPM;

  /** Creates a new hopperSubsystem. */
  public HopperSubsystem() {
    hopperMotor = new TalonFX(HopperConstants.HOPPER_MOTOR_ID);
    hopperMotor2 = new TalonFX(HopperConstants.HOPPER_MOTOR_2_ID);

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

    hopperMotor2.setControl(new Follower(hopperMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    hopper1PositionSignal = hopperMotor.getPosition(false);
    hopper1VelocitySignal = hopperMotor.getVelocity(false);
    hopper1CurrentSignal = hopperMotor.getStatorCurrent(false);
    hopper1VoltageSignal = hopperMotor.getMotorVoltage(false);

    hopper2PositionSignal = hopperMotor2.getPosition(false);
    hopper2VelocitySignal = hopperMotor2.getVelocity(false);
    hopper2CurrentSignal = hopperMotor2.getStatorCurrent(false);
    hopper2VoltageSignal = hopperMotor2.getMotorVoltage(false);
    
    hopperVelocityControl = HopperConstants.HOPPER_VELOCITY_CONTROL.clone();
  }

  public void hopperStop() {
    hopperMotor.set(0.0);
  }

  public void setHopperSpeed(double velocity) {
    // Convert mechanism rps to motor-side rps through gear reduction.
    hopperMotor.setControl(
      hopperVelocityControl.withVelocity(velocity * HopperConstants.HOPPER_GEAR_REDUCTION)
    );
  }

  /** @deprecated Use {@link #setHopperSpeed(double)}. */
  @Deprecated
  public void sethopperSpeed(double velocity) {
    setHopperSpeed(velocity);
  }

  public void publishTelemetry() {
    SmartDashboard.putNumber("Hopper/Motor1PositionRot", hopper1PositionSignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor1VelocityRps", hopper1VelocitySignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor1CurrentA", hopper1CurrentSignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor1VoltageV", hopper1VoltageSignal.getValueAsDouble());

    SmartDashboard.putNumber("Hopper/Motor2PositionRot", hopper2PositionSignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor2VelocityRps", hopper2VelocitySignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor2CurrentA", hopper2CurrentSignal.getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor2VoltageV", hopper2VoltageSignal.getValueAsDouble());
  }

  private void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        hopper1PositionSignal,
        hopper1VelocitySignal,
        hopper1CurrentSignal,
        hopper1VoltageSignal,
        hopper2PositionSignal,
        hopper2VelocitySignal,
        hopper2CurrentSignal,
        hopper2VoltageSignal);
  }

  // SIMULATION

  private DCMotor hopperDcMotor;
  private LinearSystem<N2, N1, N2> hopperSystem;
  private DCMotorSim hopperSim;

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
        }
        else {
      // Step simulation and mirror state into both hopper motors.
            final var hopperMotorSimState = hopperMotor.getSimState();
            final var hopperMotor2SimState = hopperMotor2.getSimState();

            hopperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            hopperMotor2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            
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
    hopperGoalVelocity = HopperConstants.HOPPER_FEEDING_VELOCITY.in(RotationsPerSecond);
    setHopperSpeed(hopperGoalVelocity);
  }

  public void push() {
    // Faster forward push mode for quick transfer.
    hopperGoalVelocity = HopperConstants.HOPPER_PUSHING_VELOCITY.in(RotationsPerSecond);
    setHopperSpeed(hopperGoalVelocity);
  }

  public void reverse() {
    // Reverse direction to clear jams.
    hopperGoalVelocity = HopperConstants.HOPPER_REVERSE_VELOCITY.in(RotationsPerSecond);
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
