package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

public class HopperConstants {
    public static final int HOPPER_MOTOR_ID = 41;
    public static final int HOPPER_MOTOR_2_ID = 42;
    public static final int HOPPER_SIDE_MOTOR_ID = 43;

    public static final double HOPPER_KS = 0.16645;
    public static final double HOPPER_KV = 0.11692;
    public static final double HOPPER_KA = 0.0026237;
    public static final double HOPPER_KP = 0.0079112;
    public static final double HOPPER_KI = 0.0;
    public static final double HOPPER_KD = 0.005;

    public static final TalonFXConfiguration HOPPER_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(HOPPER_KS)
                .withKV(HOPPER_KV)
                .withKP(HOPPER_KP)
                .withKI(HOPPER_KI)
                .withKD(HOPPER_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(9)
                .withPeakReverseVoltage(-9))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(37.5))
                
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));

    // Dedicated side hopper motor configuration (kept identical for now,
    // but split out so it can be tuned independently).
    public static final TalonFXConfiguration HOPPER_SIDE_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(HOPPER_KS)
                .withKV(HOPPER_KV)
                .withKP(HOPPER_KP)
                .withKI(HOPPER_KI)
                .withKD(HOPPER_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(9)
                .withPeakReverseVoltage(-9))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20))

            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive));

    public static final VelocityVoltage HOPPER_VELOCITY_CONTROL = new VelocityVoltage(0)
        .withSlot(0)
        .withEnableFOC(false);

    public static final double HOPPER_GEAR_REDUCTION = 3.2;
    public static final double HOPPER_SIDE_GEAR_REDUCTION = 24.0/12.0;

    public static final AngularVelocity HOPPER_ALLOWABLE_ERROR = RotationsPerSecond.of(1.0); // in RPS

    public static final AngularVelocity HOPPER_FEEDING_VELOCITY = RotationsPerSecond.of(13); // in RPS 20
    public static final AngularVelocity HOPPER_SIDE_PUSHING_VELOCITY = RotationsPerSecond.of(-1); // in RPS
    public static final AngularVelocity HOPPER_REVERSE_VELOCITY = RotationsPerSecond.of(-10.0); // in RPS -10

    // Hopper physical constants (defaults - adjust if needed)
    public static final Mass ROLLER_MASS = Kilograms.of(0.25); // kg
    public static final Distance ROLLER_RADIUS = Meters.of(0.015); // m
    public static final int ROLLER_AMOUNT = 12;

    public static final MomentOfInertia HOPPER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(
        0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2)).times(ROLLER_AMOUNT); // kg*m^2

}
