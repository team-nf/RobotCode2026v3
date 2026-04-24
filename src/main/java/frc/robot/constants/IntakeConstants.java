package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

public class IntakeConstants {
  // New hardware layout
  public static final int INTAKE_LINEAR_MOTOR_ID = 60;
  public static final int INTAKE_ROLLER_PRIMARY_MOTOR_ID = 61;
  public static final int INTAKE_ROLLER_SECONDARY_MOTOR_ID = 62;

  public static final double INTAKE_KS = 0.039572;
  public static final double INTAKE_KV = 0.11629;
  public static final double INTAKE_KA = 0.0042099;
  public static final double INTAKE_KP = 0.021298;
  public static final double INTAKE_KI = 0;
  public static final double INTAKE_KD = 0.0;

  public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withSlot0(
              new Slot0Configs()
                  .withKS(INTAKE_KS)
                  .withKV(INTAKE_KV)
                  .withKA(INTAKE_KA)
                  .withKP(INTAKE_KP)
                  .withKI(INTAKE_KI)
                  .withKD(INTAKE_KD))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(39)
                  .withStatorCurrentLimit(39))
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

  public static final VelocityVoltage INTAKE_VELOCITY_CONTROL =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

  // Roller reduction
  public static final double INTAKE_GEAR_REDUCTION = 20.0 / 12.0;

  // Linear rack conversion (motor-side)
  public static final Distance INTAKE_EXTENSION_PER_MOTOR_ROTATION = Millimeters.of(9.42);
  public static final double INTAKE_LINEAR_METERS_PER_MOTOR_ROTATION =
      INTAKE_EXTENSION_PER_MOTOR_ROTATION.in(Meters);

  // Linear extension targets
  public static final Distance INTAKE_EXTENSION_RETRACTED = Millimeters.of(0);
  public static final Distance INTAKE_EXTENSION_DEPLOYED = Millimeters.of(273);
  public static final Distance INTAKE_EXTENSION_FEED = Millimeters.of(120);
  public static final Distance INTAKE_EXTENSION_MAX = Millimeters.of(301);
  public static final Distance INTAKE_EXTENSION_ALLOWABLE_ERROR = Millimeters.of(2);

  // Cached primitive conversion values (avoid repeated .in(...) in runtime paths)
  public static final double INTAKE_EXTENSION_RETRACTED_METERS =
      INTAKE_EXTENSION_RETRACTED.in(Meters);
  public static final double INTAKE_EXTENSION_DEPLOYED_METERS =
      INTAKE_EXTENSION_DEPLOYED.in(Meters);
  public static final double INTAKE_EXTENSION_FEED_METERS = INTAKE_EXTENSION_FEED.in(Meters);
  public static final double INTAKE_EXTENSION_MAX_METERS = INTAKE_EXTENSION_MAX.in(Meters);
  public static final double INTAKE_EXTENSION_ALLOWABLE_ERROR_METERS =
      INTAKE_EXTENSION_ALLOWABLE_ERROR.in(Meters);

  public static double extensionMetersToMotorRotations(double meters) {
    return meters / INTAKE_LINEAR_METERS_PER_MOTOR_ROTATION;
  }

  public static double motorRotationsToExtensionMeters(double motorRotations) {
    return motorRotations * INTAKE_LINEAR_METERS_PER_MOTOR_ROTATION;
  }

  public static final AngularVelocity INTAKE_ALLOWABLE_ERROR = RotationsPerSecond.of(1.0); // in RPS
  public static final double INTAKE_ALLOWABLE_ERROR_RPS =
      INTAKE_ALLOWABLE_ERROR.in(RotationsPerSecond);

  public static final AngularVelocity INTAKE_INTAKING_VELOCITY =
      RotationsPerSecond.of(55); // in RPS
  public static final AngularVelocity INTAKE_FEEDING_VELOCITY = RotationsPerSecond.of(10); // in RPS
  public static final AngularVelocity INTAKE_REVERSE_VELOCITY =
      RotationsPerSecond.of(-15.0); // in RPS
  public static final AngularVelocity INTAKE_REVERSE_FAILSAFE_VELOCITY =
      RotationsPerSecond.of(-5.0); // in RPS
  public static final double INTAKE_INTAKING_VELOCITY_RPS =
      INTAKE_INTAKING_VELOCITY.in(RotationsPerSecond);
  public static final double INTAKE_FEEDING_VELOCITY_RPS =
      INTAKE_FEEDING_VELOCITY.in(RotationsPerSecond);
  public static final double INTAKE_REVERSE_VELOCITY_RPS =
      INTAKE_REVERSE_VELOCITY.in(RotationsPerSecond);
  public static final double INTAKE_REVERSE_FAILSAFE_VELOCITY_RPS =
      INTAKE_REVERSE_FAILSAFE_VELOCITY.in(RotationsPerSecond);

  // Compatibility fields: "ARM_*" names now represent linear position mapped as motor rotations.
  public static final Angle INTAKE_ARM_DEPLOYED_ANGLE =
      Rotations.of(extensionMetersToMotorRotations(INTAKE_EXTENSION_DEPLOYED_METERS));
  public static final Angle INTAKE_ARM_RETRACTED_ANGLE =
      Rotations.of(extensionMetersToMotorRotations(INTAKE_EXTENSION_RETRACTED_METERS));
  public static final Angle INTAKE_ARM_START_ANGLE = INTAKE_ARM_RETRACTED_ANGLE;

  public static final Angle INTAKE_FEED_ANGLE =
      Rotations.of(extensionMetersToMotorRotations(INTAKE_EXTENSION_FEED_METERS));
  public static final Angle INTAKE_ARM_BETWEEN_ANGLE = INTAKE_FEED_ANGLE;

  public static final Angle INTAKE_ARM_ALLOWABLE_ERROR =
      Rotations.of(extensionMetersToMotorRotations(INTAKE_EXTENSION_ALLOWABLE_ERROR_METERS));

  public static final Angle INTAKE_ARM_DEPLOYED_WITH_OFFSET_ANGLE =
      INTAKE_ARM_DEPLOYED_ANGLE.plus(INTAKE_ARM_ALLOWABLE_ERROR.div(8));

  public static final double INTAKE_ARM_KS = 0.04;
  public static final double INTAKE_ARM_KV = 0.13248;
  public static final double INTAKE_ARM_KA = 0.002;
  public static final double INTAKE_ARM_KP = 3.596;
  public static final double INTAKE_ARM_KI = 0;
  public static final double INTAKE_ARM_KD = 0.0;
  public static final double INTAKE_ARM_KG = 0;

  public static final TalonFXConfiguration INTAKE_ARM_MOTOR_CONFIG =
      INTAKE_MOTOR_CONFIG
          .clone()
          .withSlot0(
              new Slot0Configs()
                  .withKS(INTAKE_ARM_KS)
                  .withKV(INTAKE_ARM_KV)
                  .withKA(INTAKE_ARM_KA)
                  .withKP(INTAKE_ARM_KP)
                  .withKI(INTAKE_ARM_KI)
                  .withKD(INTAKE_ARM_KD)
                  .withKG(INTAKE_ARM_KG)
                  .withGravityType(GravityTypeValue.Arm_Cosine))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(9).withPeakReverseVoltage(-9))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(30));

  public static final PositionVoltage INTAKE_ARM_POSITION_CONTROL =
      new PositionVoltage(0).withSlot(0).withEnableFOC(false);

  public static final Mass INTAKE_ARM_MASS = Kilograms.of(1);
  public static final double INTAKE_ARM_GEAR_REDUCTION = 21.0 * 17 / 21;
  public static final double INTAKE_SIM_DRUM_RADIUS_METERS = 0.33;
  public static final Distance INTAKE_ARM_LENGTH = Meters.of(0.1);
  public static final double INTAKE_ARM_INERTIA =
      1.0 / 3.0 * INTAKE_ARM_MASS.in(Kilogram) * Math.pow(INTAKE_ARM_LENGTH.in(Meters) / 2, 2);

  // Physical defaults
  public static final Mass ROLLER_MASS = Kilograms.of(0.35); // kg
  public static final Distance ROLLER_RADIUS = Meters.of(0.02); // m
  public static final int ROLLER_AMOUNT = 2;

  public static final MomentOfInertia INTAKE_MOMENT_OF_INERTIA =
      KilogramSquareMeters.of(
              0.5 * ROLLER_MASS.in(Kilogram) * Math.pow(ROLLER_RADIUS.in(Meters), 2))
          .times(ROLLER_AMOUNT); // kg*m^2

  // Hardstop zeroing routine parameters
  public static final double INTAKE_ARM_ZEROING_REVERSE_OUTPUT = -0.12;
  public static final double INTAKE_ARM_ZEROING_STALL_VELOCITY_RPS = 0.05;
  public static final double INTAKE_ARM_ZEROING_STALL_DEBOUNCE_SEC = 0.20;
  public static final double INTAKE_ARM_ZEROING_TIMEOUT_SEC = 2.0;
}
