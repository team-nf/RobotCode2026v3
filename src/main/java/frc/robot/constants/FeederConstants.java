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

public class FeederConstants {
    public static final int FEEDER_BELT_MOTOR_ID = 31;
    public static final int FEEDER_FEED_MOTOR_ID = 32;

    // Belt motor PID/feedforward
    public static final double FEEDER_BELT_KS = 0.20536;
    public static final double FEEDER_BELT_KV = 0.11887;
    public static final double FEEDER_BELT_KA = 0.0052158;
    public static final double FEEDER_BELT_KP = 0.10035;
    public static final double FEEDER_BELT_KI = 0.0;
    public static final double FEEDER_BELT_KD = 0.0;

    // Feed wheel motor PID/feedforward (separate tune from belt)
    public static final double FEEDER_FEED_KS = 0.0;
    public static final double FEEDER_FEED_KV = 0.11795;
    public static final double FEEDER_FEED_KA = 0.021261;
    public static final double FEEDER_FEED_KP = 0.1568;
    public static final double FEEDER_FEED_KI = 0.0;
    public static final double FEEDER_FEED_KD = 0.0;

    public static final TalonFXConfiguration FEEDER_BELT_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(FEEDER_BELT_KS)
                .withKV(FEEDER_BELT_KV)
                .withKA(FEEDER_BELT_KA)
                .withKP(FEEDER_BELT_KP)
                .withKI(FEEDER_BELT_KI)
                .withKD(FEEDER_BELT_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12)
                .withPeakReverseVoltage(-12))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(37.5))
                
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));

    public static final TalonFXConfiguration FEEDER_FEED_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(FEEDER_FEED_KS)
                .withKV(FEEDER_FEED_KV)
                .withKA(FEEDER_FEED_KA)
                .withKP(FEEDER_FEED_KP)
                .withKI(FEEDER_FEED_KI)
                .withKD(FEEDER_FEED_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(9)
                .withPeakReverseVoltage(-9))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(38))
                
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive));

    public static final VelocityVoltage FEEDER_VELOCITY_CONTROL = new VelocityVoltage(0)
        .withSlot(0)
        .withEnableFOC(false);

    // Feed motor reduction
    public static final double FEEDER_GEAR_REDUCTION = 2.0;
    // Belt motor reduction
    public static final double FEEDER_BELT_GEAR_REDUCTION = 3.0;

    public static final AngularVelocity FEEDER_ALLOWABLE_ERROR = RotationsPerSecond.of(30); // in RPS
    public static final AngularVelocity FEEDER_FEEDING_VELOCITY = RotationsPerSecond.of(30); // in RPS
    public static final AngularVelocity FEEDER_FEEDING_BELT_VELOCITY = RotationsPerSecond.of(50); // in RPS
    public static final AngularVelocity FEEDER_REVERSE_VELOCITY = RotationsPerSecond.of(-2.0); // in RPS
    public static final double FEEDER_ALLOWABLE_ERROR_RPS = FEEDER_ALLOWABLE_ERROR.in(RotationsPerSecond);
    public static final double FEEDER_FEEDING_VELOCITY_RPS = FEEDER_FEEDING_VELOCITY.in(RotationsPerSecond);
    public static final double FEEDER_FEEDING_BELT_VELOCITY_RPS = FEEDER_FEEDING_BELT_VELOCITY.in(RotationsPerSecond);
    public static final double FEEDER_REVERSE_VELOCITY_RPS = FEEDER_REVERSE_VELOCITY.in(RotationsPerSecond);

    // Feeder syystem constants

    public static final Mass ROLLER_MASS = Kilograms.of(0.2); // kg
    public static final Distance ROLLER_RADIUS = Meters.of(0.015); //
    public static final int ROLLER_AMOUNT = 4;

    public static final MomentOfInertia FEEDER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(
        0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2)).times(ROLLER_AMOUNT); // kg*m^2

}
