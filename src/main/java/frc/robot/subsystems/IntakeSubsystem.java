// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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
  private double intakeGoalExtensionMeters;
  private double intakeTestRPM;
  private double intakeTestExtensionMeters;


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

    
  if(Robot.isReal()) {
    intakeArmMotor.setPosition(IntakeConstants.extensionMetersToMotorRotations(
      IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters)));
  }

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

  // --- Linear extension control ---

  public void setIntakeExtensionMeters(double extensionMeters) {
    double clampedExtension = Math.max(
      IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters),
      Math.min(extensionMeters, IntakeConstants.INTAKE_EXTENSION_MAX.in(Meters))
    );

    double motorRotations = IntakeConstants.extensionMetersToMotorRotations(clampedExtension);
    intakeArmMotor.setControl(intakeArmPositionControl.withPosition(motorRotations));
  }

  public double getIntakeExtensionMeters() {
    double motorRotations = intakeArmMotor.getPosition().getValueAsDouble();
    return IntakeConstants.motorRotationsToExtensionMeters(motorRotations);
  }

  // --- Compatibility wrappers (temporary) ---
  public void setIntakeArmPosition(double position) {
    setIntakeExtensionMeters(IntakeConstants.motorRotationsToExtensionMeters(position));
  }

  public void intakeArmZero() {
    setIntakeExtensionMeters(IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters));
  }

  public double getIntakeArmPosition() {
    return IntakeConstants.extensionMetersToMotorRotations(getIntakeExtensionMeters());
  }

  public boolean isIntakeDeployed() {
    return getIntakeExtensionMeters() >=
        IntakeConstants.INTAKE_EXTENSION_DEPLOYED.minus(IntakeConstants.INTAKE_EXTENSION_ALLOWABLE_ERROR).in(Meters);
  }

  public boolean isIntakeRetracted() {
    return getIntakeExtensionMeters() <=
        IntakeConstants.INTAKE_EXTENSION_RETRACTED.plus(IntakeConstants.INTAKE_EXTENSION_ALLOWABLE_ERROR).in(Meters);
  }

  public void publishTelemetry() {
    SmartDashboard.putNumber("Intake/RollerPositionRot", intakeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerVelocityRps", intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerCurrentA", intakeMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Intake/ExtensionMotorPositionRot", intakeArmMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/ExtensionMotorVelocityRps", intakeArmMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/ExtensionMotorCurrentA", intakeArmMotor.getStatorCurrent().getValueAsDouble());

    SmartDashboard.putNumber("Intake/ExtensionAmountMeters", getIntakeExtensionMeters());
    SmartDashboard.putNumber("Intake/ExtensionAmountMillimeters", getIntakeExtensionMeters() * 1000.0);
  }

  // --- SIMULATION ---

  private DCMotor intakeDcMotor;
  private LinearSystem<N2, N1, N2> intakeSystem;
  private DCMotorSim intakeSim;

  private DCMotor intakeArmDcMotor;
  private ElevatorSim intakeArmSim;

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
    intakeArmSim = new ElevatorSim(
          intakeArmDcMotor,
      IntakeConstants.INTAKE_ARM_GEAR_REDUCTION,
      IntakeConstants.INTAKE_ARM_MASS.in(edu.wpi.first.units.Units.Kilograms),
      0.02,
      IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters),
      IntakeConstants.INTAKE_EXTENSION_MAX.in(Meters),
          false,
      IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters),
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

    double extensionMeters = intakeArmSim.getPositionMeters();
    double extensionMetersPerSecond = intakeArmSim.getVelocityMetersPerSecond();

    double armMotorRotations = IntakeConstants.extensionMetersToMotorRotations(extensionMeters);
    double armMotorRps = IntakeConstants.extensionMetersToMotorRotations(extensionMetersPerSecond);

      intakeArmMotorSimState.setRawRotorPosition(
      edu.wpi.first.units.Units.Rotations.of(armMotorRotations));
      intakeArmMotorSimState.setRotorVelocity(
      RotationsPerSecond.of(armMotorRps));
    }
  }

  @Override
  public void simulationPeriodic() {
    simUpdate();
  }

  // --- End of simulation ---

  public void close() {
    intakeGoalVelocity = 0;
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_RETRACTED.in(Meters);
    intakeStop();
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
  }

  public void deploy() {
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED.in(Meters);
    intakeGoalVelocity = 0;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    intakeStop();
  }

  public void intake() {
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED.in(Meters);
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
    
  }

  public void intakeWithOffset() {
    intakeGoalVelocity = IntakeConstants.INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED.in(Meters);
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void feed() {
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_FEED.in(Meters);
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void reverse() {
    intakeGoalVelocity = IntakeConstants.INTAKE_REVERSE_VELOCITY.in(RotationsPerSecond);
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_DEPLOYED.in(Meters);
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    if (isIntakeDeployed()) {
      setIntakeSpeed(intakeGoalVelocity);
    }
  }

  public void idleBetween() {
    intakeGoalVelocity = IntakeConstants.INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
    intakeGoalExtensionMeters = IntakeConstants.INTAKE_EXTENSION_FEED.in(Meters);
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  public void test() {
    intakeGoalVelocity = intakeTestRPM;
    intakeGoalExtensionMeters = intakeTestExtensionMeters;
    setIntakeExtensionMeters(intakeGoalExtensionMeters);
    setIntakeSpeed(intakeGoalVelocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
