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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.ShooterCalculator;

public class ShooterSubsystem extends SubsystemBase {

    // Flywheel motors: 2 leader-follower pairs
    // Left pair: motor1 (leader) + motor2 (follower) — inverted via SHOOTER_MOTOR_OUTPUT_CONFIGS
    // Right pair: motor3 (leader) + motor4 (follower) — default orientation
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;
    private TalonFX flywheelMotor3;
    private TalonFX flywheelMotor4;

    // Hood motor
    private TalonFX hoodMotor;

    private final VelocityVoltage flywheelVelocityControlLeft;
    private final VelocityVoltage flywheelVelocityControlRight;
    private final PositionVoltage hoodPositionControl;

    // Pre-cached unit conversions
    private static final double MIN_HOOD_ROT = ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
    private static final double MAX_HOOD_ROT = ShooterConstants.MAX_HOOD_ANGLE.in(Rotations);

    private double flywheelGoalVelocity;
    private double hoodGoalPosition;
    private double flywheelTestRPM;
    private double hoodTestAngle;

    public ShooterSubsystem() {

        // Initialize flywheel motors
        flywheelMotor1 = new TalonFX(ShooterConstants.FIRST_SHOOTER_MOTOR_ID);
        flywheelMotor2 = new TalonFX(ShooterConstants.SECOND_SHOOTER_MOTOR_ID);
        flywheelMotor3 = new TalonFX(ShooterConstants.THIRD_SHOOTER_MOTOR_ID);
        flywheelMotor4 = new TalonFX(ShooterConstants.FOURTH_SHOOTER_MOTOR_ID);

        // Initialize hood motor
        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);

        // Configure left flywheel leader (motor1) — inverted config applied via SHOOTER_CONFIG + MOTOR_OUTPUT_CONFIGS
        var leftConfig = ShooterConstants.SHOOTER_CONFIG.withMotorOutput(ShooterConstants.SHOOTER_MOTOR_OUTPUT_CONFIGS);
        applyConfig(flywheelMotor1, leftConfig, "Flywheel Motor 1 (Left Leader)");

        // Motor2 follows Motor1 (same direction = aligned)
        applyConfig(flywheelMotor2, leftConfig, "Flywheel Motor 2 (Left Follower)");
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Aligned));

        var rightConfig =  ShooterConstants.SHOOTER_CONFIG.withMotorOutput(ShooterConstants.SHOOTER_MOTOR_OUTPUT_CONFIGS.withInverted(InvertedValue.CounterClockwise_Positive));
        // Configure right flywheel leader (motor3) — default orientation
        applyConfig(flywheelMotor3, rightConfig, "Flywheel Motor 3 (Right Leader)");

        // Motor4 follows Motor3 (same direction = aligned)
        applyConfig(flywheelMotor4, rightConfig, "Flywheel Motor 4 (Right Follower)");
        flywheelMotor4.setControl(new Follower(flywheelMotor3.getDeviceID(), MotorAlignmentValue.Aligned));

        // Configure hood motor
        applyConfig(hoodMotor, ShooterConstants.HOOD_CONFIG, "Hood Motor");

        // Clone control requests
        flywheelVelocityControlLeft = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        flywheelVelocityControlRight = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        hoodPositionControl = ShooterConstants.HOOD_POSITION_CONTROL.clone();
    }

    private void applyConfig(TalonFX motor, com.ctre.phoenix6.configs.TalonFXConfiguration config, String name) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs to " + name + ", error code: " + status.toString());
        }
    }

    // ===== Motor Control Helpers =====

    private void setFlywheelSpeed(double velocityRPS) {
        double motorVelocity = velocityRPS * ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
        flywheelMotor1.setControl(flywheelVelocityControlLeft.withVelocity(motorVelocity));
        flywheelMotor3.setControl(flywheelVelocityControlRight.withVelocity(motorVelocity));
    }

    private void setHoodAngle(double positionRotations) {
        double clampedRotations = Math.max(
            MIN_HOOD_ROT,
            Math.min(positionRotations, MAX_HOOD_ROT)
        );
        double motorPosition = clampedRotations * ShooterConstants.HOOD_GEAR_REDUCTION;
        hoodMotor.setControl(hoodPositionControl.withPosition(motorPosition));
    }

    private void stopFlywheel() {
        flywheelMotor1.set(0.0);
        flywheelMotor3.set(0.0);
    }

    private void stopHood() {
        hoodMotor.set(0.0);
    }

    // ===== Action Methods =====

    /** Stop all motors. */
    public void zero() {
        flywheelGoalVelocity = 0;
        hoodGoalPosition = 0;
        stopFlywheel();
        stopHood();
    }

    /** Hold flywheel at idle speed, park hood at min angle. */
    public void rest() {
        flywheelGoalVelocity = ShooterCalculator.calculateRestFlywheelSpeed();
        hoodGoalPosition = ShooterCalculator.calculateRestHoodAngle();
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
    }

    /** Set flywheel speed and hood angle with given parameters. */
    public void shoot(double velocityRPS, double hoodAngleRotations) {
        flywheelGoalVelocity = velocityRPS;
        hoodGoalPosition = hoodAngleRotations;
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
    }

    /** Set flywheel speed and hood angle for a lob pass. */
    public void pass(double velocityRPS, double hoodAngleRotations) {
        flywheelGoalVelocity = velocityRPS;
        hoodGoalPosition = hoodAngleRotations;
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
    }

    /** Direct manual control for testing. */
    public void test(double velocityRPS, double hoodAngleRotations) {
        flywheelGoalVelocity = velocityRPS;
        hoodGoalPosition = hoodAngleRotations;
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
    }

    /** Direct manual control using stored test values. */
    public void test() {
        test(flywheelTestRPM, hoodTestAngle);
    }

    /** Get the current flywheel velocity in RPS (mechanism side). */
    public double getFlywheel1Velocity() {
        return flywheelMotor1.getVelocity().getValueAsDouble() / ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    }

    public double getFlywheel2Velocity() {
        return flywheelMotor2.getVelocity().getValueAsDouble() / ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    }

    /** Get the current hood position in rotations (mechanism side). */
    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble() / ShooterConstants.HOOD_GEAR_REDUCTION;
    }

    // Pre-cached allowable error thresholds
    private static final double FLYWHEEL_ERROR_RPS = ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR.in(RotationsPerSecond);
    private static final double HOOD_ERROR_ROT = ShooterConstants.HOOD_ALLOWABLE_ERROR.in(Rotations);

    /** Check if the flywheel is at the target speed within allowable error. */
    public boolean isFlywheelAtSpeed() {
        return Math.abs(getFlywheel1Velocity() - flywheelGoalVelocity) < FLYWHEEL_ERROR_RPS
            && Math.abs(getFlywheel2Velocity() - flywheelGoalVelocity) < FLYWHEEL_ERROR_RPS;
    }

    /** Check if the hood is at the target angle within allowable error. */
    public boolean isHoodAtAngle() {
        return Math.abs(getHoodPosition() - hoodGoalPosition) < HOOD_ERROR_ROT;
    }

    /** Check if the shooter is ready to fire (flywheel at speed AND hood at angle). */
    public boolean isReadyToShoot() {
        return isFlywheelAtSpeed() && isHoodAtAngle() && getFlywheel1Velocity() > 15;
    }

    public void publishTelemetry() {
        // Implement telemetry publishing here if needed
    }

    // ===== SIMULATION =====

    private DCMotor flywheelDcMotor;
    private LinearSystem<N2, N1, N2> flywheelSystem;
    private DCMotorSim flywheelSim;

    private DCMotor hoodDcMotor;
    private SingleJointedArmSim hoodSim;

    private boolean isSimulationInitialized = false;

    public void simUpdate() {
        if (!isSimulationInitialized) {
            isSimulationInitialized = true;

            // Flywheel sim — 2 motors per side (NUMBER_OF_FLYWHEEL_MOTORS)
            flywheelDcMotor = DCMotor.getKrakenX60(ShooterConstants.NUMBER_OF_FLYWHEEL_MOTORS);
            flywheelSystem = LinearSystemId.createDCMotorSystem(
                flywheelDcMotor,
                ShooterConstants.TOTAL_MOMENT_OF_INERTIA.in(KilogramSquareMeters),
                ShooterConstants.FLYWHEEL_GEAR_REDUCTION
            );
            flywheelSim = new DCMotorSim(flywheelSystem, flywheelDcMotor);

            // Hood sim
            hoodDcMotor = DCMotor.getKrakenX60(1);
            hoodSim = new SingleJointedArmSim(
                hoodDcMotor,
                ShooterConstants.HOOD_GEAR_REDUCTION,
                ShooterConstants.HOOD_INERTIA.in(KilogramSquareMeters),
                ShooterConstants.HOOD_LENGTH.in(Meters),
                ShooterConstants.MIN_HOOD_ANGLE.in(Radians),
                ShooterConstants.MAX_HOOD_ANGLE.in(Radians),
                true,
                ShooterConstants.MIN_HOOD_ANGLE.in(Radians)
            );

            // Configure sim orientations
            // Left pair — inverted (Clockwise_Positive)
            flywheelMotor1.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            flywheelMotor1.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            flywheelMotor2.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            flywheelMotor2.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            // Right pair — default
            flywheelMotor3.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            flywheelMotor3.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            flywheelMotor4.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            flywheelMotor4.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            // Hood
            hoodMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hoodMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        } else {
            // --- Flywheel sim update ---
            final var fw1SimState = flywheelMotor1.getSimState();
            final var fw2SimState = flywheelMotor2.getSimState();
            final var fw3SimState = flywheelMotor3.getSimState();
            final var fw4SimState = flywheelMotor4.getSimState();

            fw1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fw2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fw3SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fw4SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            flywheelSim.setInputVoltage(fw1SimState.getMotorVoltage());
            flywheelSim.update(0.002);

            // All 4 flywheel motors share the same sim state
            fw1SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw1SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            fw2SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw2SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            fw3SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw3SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            fw4SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw4SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            // --- Hood sim update ---
            final var hoodSimState = hoodMotor.getSimState();

            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
            hoodSim.update(0.002);

            hoodSimState.setRawRotorPosition(
                Rotations.of(hoodSim.getAngleRads() / (2 * Math.PI) * ShooterConstants.HOOD_GEAR_REDUCTION));
            hoodSimState.setRotorVelocity(
                RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(ShooterConstants.HOOD_GEAR_REDUCTION));
        }
    }

    @Override
    public void simulationPeriodic() {
        simUpdate();
    }

    // ===== End of simulation =====

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
