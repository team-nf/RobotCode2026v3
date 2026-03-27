package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import static edu.wpi.first.units.Units.*;

import javax.security.auth.login.FailedLoginException;

public class IntakeConstants {
    public static final int INTAKE_MOTOR_ID = 59;

    public static final double INTAKE_KS = 0.5;
    public static final double INTAKE_KV = 0.0;
    public static final double INTAKE_KP = 5;
    public static final double INTAKE_KI = 1;
    public static final double INTAKE_KD = 0.005;

    public static final TalonFXConfiguration INTAKE_MOTOR_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(INTAKE_KS)
                .withKV(INTAKE_KV)
                .withKP(INTAKE_KP)
                .withKI(INTAKE_KI)
                .withKD(INTAKE_KD))

            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(9)
                .withPeakReverseVoltage(-9))

            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(39)
                .withStatorCurrentLimit(39))
                
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))    ;

    public static final VelocityVoltage INTAKE_VELOCITY_CONTROL = new VelocityVoltage(0)
                        .withSlot(0)
                        .withEnableFOC(false);

    public static final double INTAKE_GEAR_REDUCTION = 20.0/12.0;

    public static final AngularVelocity INTAKE_ALLOWABLE_ERROR = RotationsPerSecond.of(1.0); // in RPS

    public static final AngularVelocity INTAKE_INTAKING_VELOCITY = RotationsPerSecond.of(10); // in RPS
    public static final AngularVelocity INTAKE_FEEDING_VELOCITY = RotationsPerSecond.of(5); // in RPS
    public static final AngularVelocity INTAKE_REVERSE_VELOCITY = RotationsPerSecond.of(-15.0); // in RPS
    public static final AngularVelocity INTAKE_REVERSE_FAILSAFE_VELOCITY = RotationsPerSecond.of(-5.0); // in RPS
    
    // Optional arm motor constants (defaults provided). If you use a dedicated arm motor,
    // set `INTAKE_ARM_MOTOR_ID` and override `INTAKE_ARM_MOTOR_CONFIG` and
    // `INTAKE_ARM_POSITION_CONTROL` as needed.
    // Default ID of -1 indicates "no dedicated arm motor configured"; callers may
    // choose to fallback to using the intake motor instead.

     /* 
    public static final Angle INTAKE_ARM_DEPLOYED_ANGLE = Degrees.of(1);
    public static final Angle INTAKE_ARM_RETRACTED_ANGLE = Degrees.of(110);
    public static final Angle INTAKE_ARM_START_ANGLE = Degrees.of(120);
    
    public static final Angle INTAKE_FEED_ANGLE = Degrees.of(60);
    public static final Angle INTAKE_ARM_BETWEEN_ANGLE = Degrees.of(17.5);

   */
    public static final Angle INTAKE_ARM_DEPLOYED_ANGLE = Degrees.of(0);
    public static final Angle INTAKE_ARM_RETRACTED_ANGLE = Degrees.of(0);
    public static final Angle INTAKE_ARM_START_ANGLE = Degrees.of(0);
    
    public static final Angle INTAKE_FEED_ANGLE = Degrees.of(0);
    public static final Angle INTAKE_ARM_BETWEEN_ANGLE = Degrees.of(0);
    
    public static final Angle INTAKE_ARM_ALLOWABLE_ERROR = Degrees.of(10);

    public static final Angle INTAKE_ARM_DEPLOYED_WITH_OFFSET_ANGLE = INTAKE_ARM_DEPLOYED_ANGLE.plus(INTAKE_ARM_ALLOWABLE_ERROR.div(8));

    public static final double INTAKE_ARM_KS = 0.0;
    public static final double INTAKE_ARM_KV = 0.5;
    public static final double INTAKE_ARM_KP = 2;
    public static final double INTAKE_ARM_KI = 0;
    public static final double INTAKE_ARM_KD = 0.3;
    public static final double INTAKE_ARM_KG = 0;

    public static final int INTAKE_ARM_MOTOR_ID = 58;

    public static final TalonFXConfiguration INTAKE_ARM_MOTOR_CONFIG = INTAKE_MOTOR_CONFIG.clone()
                .withSlot0(new Slot0Configs()
                .withKS(INTAKE_ARM_KS)
                .withKV(INTAKE_ARM_KV)
                .withKP(INTAKE_ARM_KP)
                .withKI(INTAKE_ARM_KI)
                .withKD(INTAKE_ARM_KD)
                .withKG(INTAKE_ARM_KG)
                .withGravityType(GravityTypeValue.Arm_Cosine))
                .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
                .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(9)
                    .withPeakReverseVoltage(-9))
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(39)
                    .withStatorCurrentLimit(39));

    public static final PositionVoltage INTAKE_ARM_POSITION_CONTROL = new PositionVoltage(0)
                        .withSlot(0)
                        .withEnableFOC(false);

    // Arm physical defaults for simulation
    public static final Mass INTAKE_ARM_MASS = Kilograms.of(0.01);
    public static final Distance INTAKE_ARM_LENGTH = Meters.of(0.1);
    public static final double INTAKE_ARM_GEAR_REDUCTION = 60.0/18.0*36/14*5;
    public static final double INTAKE_ARM_INERTIA = 1.0/3.0 * INTAKE_ARM_MASS.in(Kilogram) * Math.pow(INTAKE_ARM_LENGTH.in(Meters)/2, 2);

    // Physical defaults
    public static final Mass ROLLER_MASS = Kilograms.of(0.35); // kg
    public static final Distance ROLLER_RADIUS = Meters.of(0.02); // m
    public static final int ROLLER_AMOUNT = 2;

    public static final MomentOfInertia INTAKE_MOMENT_OF_INERTIA = KilogramSquareMeters.of(
        0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2)).times(ROLLER_AMOUNT); // kg*m^2

}
