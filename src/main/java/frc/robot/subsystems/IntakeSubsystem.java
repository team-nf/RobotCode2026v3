// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private TalonFX intakeMotor;
  private TalonFX intakeArmMotor;

  private final VelocityVoltage intakeVelocityControl;
  private final PositionVoltage intakeArmPositionControl;

  private double intakeGoalVelocity;
  private double intakeGoalArmAngle;
  private double intakeTestRPM;
  private double intakeTestAngle;


  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);
    intakeArmMotor = new TalonFX(IntakeConstants.INTAKE_ARM_MOTOR_ID);

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
      status = intakeArmMotor.getConfigurator().apply(IntakeConstants.INTAKE_ARM_MOTOR_CONFIG);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply intake arm motor configs, error code: " + status.toString());
    }

    
    if(Robot.isReal()) intakeArmMotor.setPosition(
        IntakeConstants.INTAKE_ARM_START_ANGLE.times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));

    intakeVelocityControl = IntakeConstants.INTAKE_VELOCITY_CONTROL.clone();
    intakeArmPositionControl = IntakeConstants.INTAKE_ARM_POSITION_CONTROL.clone();
  }

  // --- Roller control ---

  public void intakeStop() {
    intakeMotor.set(0.0);
  }

  public void setIntakeSpeed(double velocity) {
    intakeMotor.setControl(
      intakeVelocityControl.withVelocity(velocity * IntakeConstants.INTAKE_GEAR_REDUCTION)
    );
  }

  // --- Arm control ---

  public void setIntakeArmPosition(double position) {
    intakeArmMotor.setControl(
      intakeArmPositionControl.withPosition(position * IntakeConstants.INTAKE_ARM_GEAR_REDUCTION)
    );
  }

  public void intakeArmZero() {
    setIntakeArmPosition(IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.in(Rotations));
  }

  public double getIntakeArmPosition() {
    return intakeArmMotor.getPosition().getValue()
        .div(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION).in(Rotations);
  }

  public boolean isIntakeDeployed() {
    return getIntakeArmPosition() <= IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE
        .plus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR).in(Rotations);
  }

  public boolean isIntakeRetracted() {
    return getIntakeArmPosition() >= IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE
        .minus(IntakeConstants.INTAKE_ARM_ALLOWABLE_ERROR).in(Rotations);
  }

  public void publishTelemetry() {
    //SmartDashboard.putNumber("Intake/Intake Arm Error", intakeArmMotor.getClosedLoopError().getValueAsDouble());
    //SmartDashboard.putNumber("Intake/Intake Arm Position", getIntakeArmPosition());
    //SmartDashboard.putNumber("Intake/Intake Arm Setpoint", intakeArmMotor.getClosedLoopReference().getValueAsDouble());
    //SmartDashboard.putNumber("Intake/Intake Arm Voltage", intakeArmMotor.getMotorVoltage().getValueAsDouble());
    //SmartDashboard.putNumber("Intake/Intake Arm Current", intakeArmMotor.getStatorCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Intake/Intake Roller Velocity", intakeMotor.getVelocity().getValueAsDouble());
  }

  // --- SIMULATION ---

  private DCMotor intakeDcMotor;
  private LinearSystem<N2, N1, N2> intakeSystem;
  private DCMotorSim intakeSim;

  private DCMotor intakeArmDcMotor;
  private SingleJointedArmSim intakeArmSim;

  private boolean isSimulationInitialized = false;

  public void simUpdate() {
    if (!isSimulationInitialized) {
      isSimulationInitialized = true;

      // Roller sim
      intakeDcMotor = DCMotor.getKrakenX60(1);
      intakeSystem = LinearSystemId.createDCMotorSystem(
          intakeDcMotor,
          IntakeConstants.INTAKE_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
          IntakeConstants.INTAKE_GEAR_REDUCTION);
      intakeSim = new DCMotorSim(intakeSystem, intakeDcMotor);
      intakeMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
      intakeMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

      // Arm sim
      intakeArmDcMotor = DCMotor.getKrakenX60(1);
      intakeArmSim = new SingleJointedArmSim(
          intakeArmDcMotor,
          IntakeConstants.INTAKE_ARM_GEAR_REDUCTION,
          IntakeConstants.INTAKE_ARM_INERTIA,
          IntakeConstants.INTAKE_ARM_LENGTH.in(Meters),
          IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Radians),
          IntakeConstants.INTAKE_ARM_START_ANGLE.in(Radians),
          false,
          IntakeConstants.INTAKE_ARM_START_ANGLE.in(Radians),
          0.0,
          0.0);
      intakeArmMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
      intakeArmMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
    } else {
      final var intakeMotorSimState = intakeMotor.getSimState();
      final var intakeArmMotorSimState = intakeArmMotor.getSimState();

      intakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      intakeArmMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

      // Roller sim update
      intakeSim.setInputVoltage(intakeMotorSimState.getMotorVoltage());
      intakeSim.update(0.002);

      intakeMotorSimState.setRawRotorPosition(
          intakeSim.getAngularPosition().times(IntakeConstants.INTAKE_GEAR_REDUCTION));
      intakeMotorSimState.setRotorVelocity(
          intakeSim.getAngularVelocity().times(IntakeConstants.INTAKE_GEAR_REDUCTION));

      // Arm sim update
      intakeArmSim.setInput(intakeArmMotorSimState.getMotorVoltage());
      intakeArmSim.update(0.002);

      intakeArmMotorSimState.setRawRotorPosition(
          Radians.of(intakeArmSim.getAngleRads()).times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));
      intakeArmMotorSimState.setRotorVelocity(
          RadiansPerSecond.of(intakeArmSim.getVelocityRadPerSec()).times(IntakeConstants.INTAKE_ARM_GEAR_REDUCTION));
    }
  }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // --- End of simulation ---

  public void close() {
    intakeGoalVelocity = 0;
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_RETRACTED_ANGLE.in(Rotations);
    intakeStop();
    intakeArmZero();
  }

  public void deploy() {
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    intakeGoalVelocity = 0;
    setIntakeArmPosition(intakeGoalArmAngle);
    intakeStop();
  }

  public void intake() {
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    setIntakeArmPosition(intakeGoalArmAngle);
    setIntakeSpeed(intakeGoalVelocity);
    
  }

  public void intakeWithOffset() {
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_WITH_OFFSET_ANGLE.in(Rotations);
    setIntakeArmPosition(intakeGoalArmAngle);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void feed() {
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeGoalArmAngle = IntakeConstants.INTAKE_FEED_ANGLE.in(Rotations);
    setIntakeArmPosition(intakeGoalArmAngle);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void reverse() {
    intakeGoalVelocity = IntakeConstants.INTAKE_REVERSE_VELOCITY.in(RotationsPerSecond);
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_DEPLOYED_ANGLE.in(Rotations);
    setIntakeArmPosition(intakeGoalArmAngle);
    if (isIntakeDeployed()) {
      setIntakeSpeed(intakeGoalVelocity);
    }
  }

  public void idleBetween() {
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeGoalArmAngle = IntakeConstants.INTAKE_ARM_BETWEEN_ANGLE.in(Rotations);
    setIntakeArmPosition(intakeGoalArmAngle);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void test() {
    intakeGoalVelocity = intakeTestRPM;
    intakeGoalArmAngle = intakeTestAngle;
    setIntakeArmPosition(intakeGoalArmAngle);
    setIntakeSpeed(intakeGoalVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
