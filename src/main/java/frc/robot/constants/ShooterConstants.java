package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.*;
import frc.robot.Robot;

public class ShooterConstants {
    // Motor IDs
    public static final int FIRST_SHOOTER_MOTOR_ID = 51;
    public static final int SECOND_SHOOTER_MOTOR_ID = 52;
    public static final int HOOD_MOTOR_ID = 55;
    public static final int TURRET_MOTOR_ID = 56;
    // Assumption: next free CAN ID is 57 for turret absolute encoder.
    public static final int TURRET_CANCODER_ID = 57;

    // Shooter Math Constants
    public static final double SHOOTER_VELOCITY_TRANSFER_COEFFICIENT = 0.81; // in meters (2 inches)
    public static final double SHOOTER_VELOCITY_SIM_TRANSFER_COEFFICIENT = 0.675; // in meters (2 inches)

    // Configs
    public static final int NUMBER_OF_FLYWHEEL_MOTORS = 2;

    public static final AngularVelocity FLYWHEEL_ALLOWABLE_ERROR = RotationsPerSecond.of(1.5); // Allowable error in radians per second
    public static final Angle HOOD_ALLOWABLE_ERROR = Degrees.of(1.5); // Allowable error in radians
    public static final Angle TURRET_ALLOWABLE_ERROR = Degrees.of(4);
    public static final AngularVelocity TURRET_ALLOWABLE_SPEED_TO_SHOOT = RotationsPerSecond.of(15);

    public static final double TURRET_LOOKAHEAD_SEC = 0.1;

    public static final double SHOOTER_KS = 0.05;
    public static final double SHOOTER_KV = 0.115;
    public static final double SHOOTER_KP = 0.0725;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.001;
    public static final double SHOOTER_KA = 0.005; 

    public static final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(SHOOTER_KS)
                .withKV(SHOOTER_KV)
                .withKP(SHOOTER_KP)
                .withKI(SHOOTER_KI)
                .withKD(SHOOTER_KD)
                .withKA(SHOOTER_KA))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(10)
                .withPeakReverseVoltage(-10))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(38)
                .withStatorCurrentLimit(38));

    public static final VelocityVoltage SHOOTER_VELOCITY_CONTROL = new VelocityVoltage(0)
        .withSlot(0)
        .withEnableFOC(false);

    public static final double HOOD_KS = 0.0;
    public static final double HOOD_KV = 0.0;
    public static final double HOOD_KP = 8.0;
    public static final double HOOD_KI = 0.0;
    public static final double HOOD_KD = 0.2;
    public static final double HOOD_KG = 0.0;

    public static final TalonFXConfiguration HOOD_CONFIG = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKS(HOOD_KS)
                .withKV(HOOD_KV)
                .withKP(HOOD_KP)
                .withKI(HOOD_KI)
                .withKD(HOOD_KD)
                .withKG(HOOD_KG)
                .withGravityType(GravityTypeValue.Arm_Cosine))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(10)
                .withPeakReverseVoltage(-10))
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(38)
                .withStatorCurrentLimit(38))
            .withMotorOutput(new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast));

    public static final PositionVoltage HOOD_POSITION_CONTROL = new PositionVoltage(0)
        .withSlot(0)
        .withEnableFOC(false);

    public static final double TURRET_AGGRESSIVE_KS = 1.0;
    public static final double TURRET_AGGRESSIVE_KV = 10.0;
    public static final double TURRET_AGGRESSIVE_KP = 60.0;
    public static final double TURRET_AGGRESSIVE_KI = 0.0;
    public static final double TURRET_AGGRESSIVE_KD = 0.0;
    public static final double TURRET_AGGRESSIVE_KA = 30.0;


    public static final double TURRET_GENTLE_KS = 1;
    public static final double TURRET_GENTLE_KV = 0.0;
    public static final double TURRET_GENTLE_KP = 8.0;
    public static final double TURRET_GENTLE_KI = 1;
    public static final double TURRET_GENTLE_KD = 0.01;

    public static final double TURRET_SMALL_ERROR_THRESHOLD_DEG = 16.0;

    public static final int TURRET_AGGRESSIVE_SLOT = 0;
    public static final int TURRET_GENTLE_SLOT = 1;

    public static final TalonFXConfiguration TURRET_CONFIG = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
            .withKS(TURRET_AGGRESSIVE_KS)
            .withKV(TURRET_AGGRESSIVE_KV)
            .withKP(TURRET_AGGRESSIVE_KP)
            .withKI(TURRET_AGGRESSIVE_KI)
            .withKD(TURRET_AGGRESSIVE_KD)
            .withKA(TURRET_AGGRESSIVE_KA))
        .withSlot1(new Slot1Configs()
            .withKS(TURRET_GENTLE_KS)
            .withKV(TURRET_GENTLE_KV)
            .withKP(TURRET_GENTLE_KP)
            .withKI(TURRET_GENTLE_KI)
            .withKD(TURRET_GENTLE_KD))
        .withVoltage(new VoltageConfigs()
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12))
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(39)
            .withStatorCurrentLimit(39))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake));

    public static final PositionVoltage TURRET_POSITION_CONTROL = new PositionVoltage(0)
        .withSlot(0)
        .withEnableFOC(false);


    // Physical Constants
    public static final double FLYWHEEL_GEAR_REDUCTION = 1;
    public static final double HOOD_GEAR_REDUCTION = 324.0/20.0*26/18*56/8;
    
    public static final double TURRET_GEAR_REDUCTION = (Robot.isReal()) ? 36.44 : 5;

    public static final double TURRET_ABSOLUTE_DEGREES_PER_ENCODER_ROTATION = 45.0;
    public static final double TURRET_ABSOLUTE_OFFSET_DEGREES = 0.0;
    public static final Angle MIN_TURRET_ANGLE = Degrees.of(-350);
    public static final Angle MAX_TURRET_ANGLE = Degrees.of(350);

    private static final Mass FLYWHEEL_MASS = Kilogram.of(0.1); 
    public static final Distance FLYWHEEL_RADIUS = Meters.of(0.05); // Radius of the flywheel in meters
    private static final Mass ROLLER_MASS = Kilogram.of(0.1); // Mass of the hood in kg
    private static final Distance ROLLER_RADIUS = Meters.of(0.0175); // Radius of the hood in meters

    public static final MomentOfInertia FLYWHEEL_MOMENT_OF_INERTIA = 
                KilogramSquareMeters.of(2*0.5*FLYWHEEL_MASS.in(Kilogram)*Math.pow(FLYWHEEL_RADIUS.in(Meters), 2));
            
    public static final MomentOfInertia HOOD_MOMENT_OF_INERTIA = 
                KilogramSquareMeters.of(2*0.5*ROLLER_MASS.in(Kilogram)*Math.pow(ROLLER_RADIUS.in(Meters), 2));
    public static final MomentOfInertia TOTAL_MOMENT_OF_INERTIA = FLYWHEEL_MOMENT_OF_INERTIA.plus(HOOD_MOMENT_OF_INERTIA);

    public static final Angle MIN_HOOD_ANGLE = Degrees.of(0);
    public static final Angle MAX_HOOD_ANGLE = Degrees.of(20);
    public static final Angle PASS_HOOD_ANGLE = Degrees.of(10);
    public static final Angle HOOD_ANGLE_OFFSET = Degrees.of(18);

    public static final Angle MIN_HOOD_MOTOR_ANGLE = MIN_HOOD_ANGLE.times(HOOD_GEAR_REDUCTION);
    public static final Angle MAX_HOOD_MOTOR_ANGLE = MAX_HOOD_ANGLE.times(HOOD_GEAR_REDUCTION);

    public static final Mass HOOD_MASS = Kilogram.of(1.5);
    public static final Distance HOOD_LENGTH = Meters.of(0.075);
    public static final Distance HOOD_CENTER_OF_MASS = HOOD_LENGTH.times(0.5);
    public static final MomentOfInertia HOOD_INERTIA = 
        KilogramSquareMeters.of(HOOD_MASS.in(Kilogram) * Math.pow(HOOD_LENGTH.in(Meters), 2) / 3.0);

    public static final MomentOfInertia TURRET_INERTIA = KilogramSquareMeters.of(0.001);

    public static final AngularVelocity MIN_FLYWHEEL_SPEED = RotationsPerSecond.of(500/60); // in RPS
    public static final AngularVelocity MAX_FLYWHEEL_SPEED = RotationsPerSecond.of(3750/60); // in RPS
    public static final AngularVelocity FLYWHEEL_REST_SPEED = RotationsPerSecond.of(750/60); // in RPS

}