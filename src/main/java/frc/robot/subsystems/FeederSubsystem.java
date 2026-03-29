// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {

  private TalonFX feederMotor;

  private final VelocityVoltage feederVelocityControl;
  private final StatusSignal<?> feederPositionSignal;
  private final StatusSignal<?> feederVelocitySignal;
  private final StatusSignal<?> feederCurrentSignal;
  private final StatusSignal<?> feederVoltageSignal;

  private double feederGoalVelocity;
  private double feederTestRPM;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    feederMotor = new TalonFX(FeederConstants.FEEDER_MOTOR_ID);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = feederMotor.getConfigurator().apply(FeederConstants.FEEDER_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    feederPositionSignal = feederMotor.getPosition(false);
    feederVelocitySignal = feederMotor.getVelocity(false);
    feederCurrentSignal = feederMotor.getStatorCurrent(false);
    feederVoltageSignal = feederMotor.getMotorVoltage(false);
    
    feederVelocityControl = FeederConstants.FEEDER_VELOCITY_CONTROL.clone();
  }

  public void feederStop() {
    feederMotor.set(0.0);
  }

  public void setFeederSpeed(double velocity) {
    feederMotor.setControl(
      feederVelocityControl.withVelocity(velocity * FeederConstants.FEEDER_GEAR_REDUCTION)
    );
  }

  public void publishTelemetry() {
    SmartDashboard.putNumber("Feeder/MotorPositionRot", feederPositionSignal.getValueAsDouble());
    SmartDashboard.putNumber("Feeder/MotorVelocityRps", feederVelocitySignal.getValueAsDouble());
    SmartDashboard.putNumber("Feeder/MotorCurrentA", feederCurrentSignal.getValueAsDouble());
    SmartDashboard.putNumber("Feeder/MotorVoltageV", feederVoltageSignal.getValueAsDouble());
  }

  private void refreshStatusSignals() {
    BaseStatusSignal.refreshAll(
        feederPositionSignal,
        feederVelocitySignal,
        feederCurrentSignal,
        feederVoltageSignal);
  }

  // SIMULATION

  private DCMotor feederDcMotor;
  private LinearSystem<N2, N1, N2> feederSystem;
  private DCMotorSim feederSim;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {

        if (!isSimulationInitialized) {
            isSimulationInitialized = true;
            feederDcMotor = DCMotor.getKrakenX60(1);
            feederSystem = LinearSystemId.createDCMotorSystem(
            feederDcMotor,
            FeederConstants.FEEDER_MOMENT_OF_INERTIA.in(KilogramSquareMeters), // Moment of Inertia
            FeederConstants.FEEDER_GEAR_REDUCTION);

            feederSim = new DCMotorSim(feederSystem, feederDcMotor);
            feederMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            feederMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
        }
        else {
            final var feederMotorSimState = feederMotor.getSimState();

            feederMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            
            feederSim.setInputVoltage(feederMotorSimState.getMotorVoltage());
            feederSim.update(0.002);

            feederMotorSimState.setRawRotorPosition(
                feederSim.getAngularPosition().times(FeederConstants.FEEDER_GEAR_REDUCTION));

            feederMotorSimState.setRotorVelocity(
                feederSim.getAngularVelocity().times(FeederConstants.FEEDER_GEAR_REDUCTION));
        }   
    }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // End of simulation part

  public void zero() {
    feederGoalVelocity = 0;
    feederStop();
  }

  public void feed() {
    feederGoalVelocity = FeederConstants.FEEDER_FEEDING_VELOCITY.in(RotationsPerSecond);
    setFeederSpeed(feederGoalVelocity);
  }

  public void reverse() {
    feederGoalVelocity = FeederConstants.FEEDER_REVERSE_VELOCITY.in(RotationsPerSecond);
    setFeederSpeed(feederGoalVelocity);
  }

  public void test() {
    feederGoalVelocity = feederTestRPM;
    setFeederSpeed(feederGoalVelocity);
  }


  @Override
  public void periodic() {
    refreshStatusSignals();
  }
}
