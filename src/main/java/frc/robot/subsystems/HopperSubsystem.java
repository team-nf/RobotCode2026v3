// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private TalonFX hopperMotor;
  private TalonFX hopperMotor2;


  private final VelocityVoltage hopperVelocityControl;

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
    
    hopperVelocityControl = HopperConstants.HOPPER_VELOCITY_CONTROL.clone();
  }

  public void hopperStop() {
    hopperMotor.set(0.0);
  }

  public void sethopperSpeed(double velocity) {
    hopperMotor.setControl(
      hopperVelocityControl.withVelocity(velocity * HopperConstants.HOPPER_GEAR_REDUCTION)
    );
  }

  public void publishTelemetry() {
    // Implement telemetry publishing here if needed
  }

  // SIMULATION

  private DCMotor hopperDcMotor;
  private LinearSystem<N2, N1, N2> hopperSystem;
  private DCMotorSim hopperSim;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {

        if (!isSimulationInitialized) {
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
    hopperGoalVelocity = 0;
    hopperStop();
  }

  public void feed() {
    hopperGoalVelocity = HopperConstants.HOPPER_FEEDING_VELOCITY.in(RotationsPerSecond);
    sethopperSpeed(hopperGoalVelocity);
  }

  public void push() {
    hopperGoalVelocity = HopperConstants.HOPPER_PUSHING_VELOCITY.in(RotationsPerSecond);
    sethopperSpeed(hopperGoalVelocity);
  }

  public void reverse() {
    hopperGoalVelocity = HopperConstants.HOPPER_REVERSE_VELOCITY.in(RotationsPerSecond);
    sethopperSpeed(hopperGoalVelocity);
  }

  public void test() {
    hopperGoalVelocity = hopperTestRPM;
    sethopperSpeed(hopperGoalVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
