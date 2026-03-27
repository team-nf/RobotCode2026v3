// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
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
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.ShooterCalculator;

public class ShooterSubsystem extends SubsystemBase {

    // Flywheel motors: motor1 leader + motor2 follower
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    private TalonFX hoodMotor;
    private TalonFX turretMotor;
    private CANcoder turretAbsoluteEncoder;

    private final VelocityVoltage flywheelVelocityControl;
    private final PositionVoltage hoodPositionControl;
    private final PositionVoltage turretPositionControl;

    private static final double MIN_HOOD_ROT = ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
    private static final double MAX_HOOD_ROT = ShooterConstants.MAX_HOOD_ANGLE.in(Rotations);
    private static final double MIN_TURRET_DEG = ShooterConstants.MIN_TURRET_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    private static final double MAX_TURRET_DEG = ShooterConstants.MAX_TURRET_ANGLE.in(edu.wpi.first.units.Units.Degrees);

    private double flywheelGoalVelocity;
    private double hoodGoalPosition;
    private double turretGoalAngleDegrees;
    private double flywheelTestRPM;
    private double hoodTestAngle;
    private double turretTestAngleDegrees;

    public ShooterSubsystem() {

        flywheelMotor1 = new TalonFX(ShooterConstants.FIRST_SHOOTER_MOTOR_ID);
        flywheelMotor2 = new TalonFX(ShooterConstants.SECOND_SHOOTER_MOTOR_ID);
        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
        turretMotor = new TalonFX(ShooterConstants.TURRET_MOTOR_ID);
        turretAbsoluteEncoder = new CANcoder(ShooterConstants.TURRET_CANCODER_ID);

        applyConfig(flywheelMotor1, ShooterConstants.SHOOTER_CONFIG, "Flywheel Motor 1 (Leader)");
        applyConfig(flywheelMotor2, ShooterConstants.SHOOTER_CONFIG, "Flywheel Motor 2 (Follower)");
    flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        applyConfig(hoodMotor, ShooterConstants.HOOD_CONFIG, "Hood Motor");
        applyConfig(turretMotor, ShooterConstants.TURRET_CONFIG, "Turret Motor");

        flywheelVelocityControl = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        hoodPositionControl = ShooterConstants.HOOD_POSITION_CONTROL.clone();
        turretPositionControl = ShooterConstants.TURRET_POSITION_CONTROL.clone();

        syncTurretMotorToAbsoluteEncoder();
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
        flywheelMotor1.setControl(flywheelVelocityControl.withVelocity(motorVelocity));
    }

    private void setHoodAngle(double positionRotations) {
        double clampedRotations = Math.max(
            MIN_HOOD_ROT,
            Math.min(positionRotations, MAX_HOOD_ROT)
        );
        double motorPosition = clampedRotations * ShooterConstants.HOOD_GEAR_REDUCTION;
        hoodMotor.setControl(hoodPositionControl.withPosition(motorPosition));
    }

    private double setTurretAngleDegrees(double requestedAngleDegrees) {
        double wrappedTargetDeg = wrapTurretTargetToCurrent(requestedAngleDegrees);
        double currentAngleDeg = getTurretAngleDegrees();
        double angleErrorDeg = Math.abs(normalizeToMinus180To180(wrappedTargetDeg - currentAngleDeg));
        int turretPidSlot = angleErrorDeg <= ShooterConstants.TURRET_SMALL_ERROR_THRESHOLD_DEG
            ? ShooterConstants.TURRET_AGGRESSIVE_SLOT
            : ShooterConstants.TURRET_GENTLE_SLOT;

        double motorPosition = wrappedTargetDeg / 360.0 * ShooterConstants.TURRET_GEAR_REDUCTION;
        turretMotor.setControl(
            turretPositionControl
                .withSlot(turretPidSlot)
                .withPosition(motorPosition)
        );
        return wrappedTargetDeg;
    }

    private double normalizeToMinus180To180(double angleDeg) {
        double wrapped = angleDeg % 360.0;
        if (wrapped > 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    private double clampTurretRange(double angleDeg) {
        return Math.max(MIN_TURRET_DEG, Math.min(angleDeg, MAX_TURRET_DEG));
    }

    private double wrapTurretTargetToCurrent(double requestedAngleDegrees) {
        double requestedClamped = clampTurretRange(requestedAngleDegrees);
        double currentAngle = getTurretAngleDegrees();

        double delta = normalizeToMinus180To180(requestedClamped - currentAngle);
        double nearestEquivalent = currentAngle + delta;

        if (nearestEquivalent < MIN_TURRET_DEG) {
            nearestEquivalent += 360.0;
        } else if (nearestEquivalent > MAX_TURRET_DEG) {
            nearestEquivalent -= 360.0;
        }

        return clampTurretRange(nearestEquivalent);
    }

    private void stopFlywheel() {
        flywheelMotor1.set(0.0);
    }

    public void syncTurretMotorToAbsoluteEncoder() {
        double absoluteEncoderRot = turretAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        double absoluteTurretDeg = absoluteEncoderRot
                * ShooterConstants.TURRET_ABSOLUTE_DEGREES_PER_ENCODER_ROTATION
                + ShooterConstants.TURRET_ABSOLUTE_OFFSET_DEGREES;
        absoluteTurretDeg = clampTurretRange(absoluteTurretDeg);

        double turretMotorRot = absoluteTurretDeg / 360.0 * ShooterConstants.TURRET_GEAR_REDUCTION;
        turretMotor.setPosition(turretMotorRot);
        turretGoalAngleDegrees = absoluteTurretDeg;
    }

    // ===== Action Methods =====

    /** Stop all motors. */
    public void zero() {
        flywheelGoalVelocity = 0;
        hoodGoalPosition = ShooterCalculator.calculateRestHoodAngle();
        turretGoalAngleDegrees = 0;
        stopFlywheel();
        setHoodAngle(hoodGoalPosition);
        turretGoalAngleDegrees = setTurretAngleDegrees(turretGoalAngleDegrees);
    }

    /** Hold flywheel at idle speed, park hood at min angle. */
    public void rest() {
        flywheelGoalVelocity = ShooterCalculator.calculateRestFlywheelSpeed();
        hoodGoalPosition = ShooterCalculator.calculateRestHoodAngle();
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
    }

    /** Set flywheel speed + hood angle + turret angle (degrees). */
    public void shoot(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        flywheelGoalVelocity = velocityRPS;
        hoodGoalPosition = hoodAngleRotations;
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalPosition);
        turretGoalAngleDegrees = setTurretAngleDegrees(turretAngleDegrees);
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void shoot(double velocityRPS, double hoodAngleRotations) {
        shoot(velocityRPS, hoodAngleRotations, 0.0);
    }

    /** Set flywheel speed + hood angle + turret angle (degrees) for pass. */
    public void pass(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void pass(double velocityRPS, double hoodAngleRotations) {
        pass(velocityRPS, hoodAngleRotations, 0.0);
    }

    /** Direct manual control for testing. */
    public void test(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void test(double velocityRPS, double hoodAngleRotations) {
        test(velocityRPS, hoodAngleRotations, 0.0);
    }

    /** Direct manual control using stored test values. */
    public void test() {
        test(flywheelTestRPM, hoodTestAngle, turretTestAngleDegrees);
    }

    /** Get the current flywheel velocity in RPS (mechanism side). */
    public double getFlywheel1Velocity() {
        return flywheelMotor1.getVelocity().getValueAsDouble() / ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    }

    public double getFlywheel1SpeedAbs() {
        return Math.abs(getFlywheel1Velocity());
    }

    /** Get the current hood position in rotations (mechanism side). */
    public double getHoodPosition() {
        return hoodMotor.getPosition().getValueAsDouble() / ShooterConstants.HOOD_GEAR_REDUCTION;
    }

    public double getTurretAngleDegrees() {
        return turretMotor.getPosition().getValueAsDouble() / ShooterConstants.TURRET_GEAR_REDUCTION * 360.0;
    }

    // Pre-cached allowable error thresholds
    private static final double FLYWHEEL_ERROR_RPS = ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR.in(RotationsPerSecond);
    private static final double HOOD_ERROR_ROT = ShooterConstants.HOOD_ALLOWABLE_ERROR.in(Rotations);
    private static final double TURRET_ERROR_DEG = ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);

    /** Check if the flywheel is at the target speed within allowable error. */
    public boolean isFlywheelAtSpeed() {
        return Math.abs(getFlywheel1SpeedAbs() - Math.abs(flywheelGoalVelocity)) < FLYWHEEL_ERROR_RPS;
    }

    /** Check if the hood is at the target angle within allowable error. */
    public boolean isHoodAtAngle() {
        return Math.abs(getHoodPosition() - hoodGoalPosition) < HOOD_ERROR_ROT;
    }

    public boolean isTurretAtAngle() {
        return Math.abs(getTurretAngleDegrees() - turretGoalAngleDegrees) < TURRET_ERROR_DEG;
    }

    /** Check if the shooter is ready to fire (flywheel + hood + turret). */
    public boolean isReadyToShoot() {
        return isFlywheelAtSpeed() && isHoodAtAngle() && isTurretAtAngle() && getFlywheel1SpeedAbs() > 15;
    }

    public void publishTelemetry() {
        double turretAngleClamped = Math.max(MIN_TURRET_DEG, Math.min(getTurretAngleDegrees(), MAX_TURRET_DEG));
        double flywheelRps = getFlywheel1SpeedAbs();
        double flywheelRpmError = (Math.abs(flywheelGoalVelocity) - flywheelRps) * 60.0;
        double hoodDegError = (hoodGoalPosition - getHoodPosition()) * 360.0;
        double turretDegError = turretGoalAngleDegrees - getTurretAngleDegrees();

        SmartDashboard.putNumber("Shooter/Flywheel1VelocityRps", flywheelMotor1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel1CurrentA", flywheelMotor1.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel1VoltageV", flywheelMotor1.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/Flywheel2VelocityRps", flywheelMotor2.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel2CurrentA", flywheelMotor2.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Flywheel2VoltageV", flywheelMotor2.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/HoodMotorPositionRot", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/HoodMotorVelocityRps", hoodMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/HoodMotorCurrentA", hoodMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/HoodMotorVoltageV", hoodMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/TurretMotorPositionRot", turretMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TurretMotorVelocityRps", turretMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TurretMotorCurrentA", turretMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TurretMotorVoltageV", turretMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Shooter/FlywheelRPS", flywheelRps);
        SmartDashboard.putNumber("Shooter/HoodAngleRot", getHoodPosition());
        SmartDashboard.putNumber("Shooter/HoodAngleDeg", getHoodPosition() * 360.0);
        SmartDashboard.putNumber("Shooter/TurretAngleDeg", turretAngleClamped);
        SmartDashboard.putNumber("Shooter/TurretGoalDeg", turretGoalAngleDegrees);
        SmartDashboard.putNumber("Shooter/FlywheelRpmError", flywheelRpmError);
        SmartDashboard.putNumber("Shooter/HoodDegError", hoodDegError);
        SmartDashboard.putNumber("Shooter/TurretDegError", turretDegError);
        SmartDashboard.putBoolean("Shooter/FlywheelReady", isFlywheelAtSpeed());
        SmartDashboard.putBoolean("Shooter/HoodReady", isHoodAtAngle());
        SmartDashboard.putBoolean("Shooter/TurretReady", isTurretAtAngle());
        SmartDashboard.putBoolean("Shooter/Ready", isReadyToShoot());
    }

    // ===== SIMULATION =====

    private DCMotor flywheelDcMotor;
    private LinearSystem<N2, N1, N2> flywheelSystem;
    private DCMotorSim flywheelSim;

    private DCMotor hoodDcMotor;
    private SingleJointedArmSim hoodSim;

    private DCMotor turretDcMotor;
    private LinearSystem<N2, N1, N2> turretSystem;
    private DCMotorSim turretSim;

    private boolean isSimulationInitialized = false;

    public void simUpdate() {
        if (!isSimulationInitialized) {
            isSimulationInitialized = true;

            // Flywheel sim — 2 motors (leader + follower)
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
                ShooterConstants.MIN_HOOD_ANGLE.in(edu.wpi.first.units.Units.Radians),
                ShooterConstants.MAX_HOOD_ANGLE.in(edu.wpi.first.units.Units.Radians),
                true,
                ShooterConstants.MIN_HOOD_ANGLE.in(edu.wpi.first.units.Units.Radians)
            );

            turretDcMotor = DCMotor.getKrakenX60(1);
            turretSystem = LinearSystemId.createDCMotorSystem(
                turretDcMotor,
                ShooterConstants.TURRET_INERTIA.in(KilogramSquareMeters),
                ShooterConstants.TURRET_GEAR_REDUCTION
            );
            turretSim = new DCMotorSim(turretSystem, turretDcMotor);

            // Configure sim orientations
            flywheelMotor1.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            flywheelMotor1.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
            flywheelMotor2.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            flywheelMotor2.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            // Hood
            hoodMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
            hoodMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

            turretMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
            turretMotor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);

        } else {
            // --- Flywheel sim update ---
            final var fw1SimState = flywheelMotor1.getSimState();
            final var fw2SimState = flywheelMotor2.getSimState();

            fw1SimState.setSupplyVoltage(RobotController.getBatteryVoltage());
            fw2SimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            flywheelSim.setInputVoltage(fw1SimState.getMotorVoltage());
            flywheelSim.update(0.002);

            fw1SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw1SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            fw2SimState.setRawRotorPosition(
                flywheelSim.getAngularPosition().times(-ShooterConstants.FLYWHEEL_GEAR_REDUCTION));
            fw2SimState.setRotorVelocity(
                flywheelSim.getAngularVelocity().times(-ShooterConstants.FLYWHEEL_GEAR_REDUCTION));

            // --- Hood sim update ---
            final var hoodSimState = hoodMotor.getSimState();

            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
            hoodSim.update(0.002);

            hoodSimState.setRawRotorPosition(
                Rotations.of(hoodSim.getAngleRads() / (2 * Math.PI) * ShooterConstants.HOOD_GEAR_REDUCTION));
            hoodSimState.setRotorVelocity(
                RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

            // --- Turret sim update ---
            final var turretSimState = turretMotor.getSimState();

            turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            turretSim.setInputVoltage(turretSimState.getMotorVoltage());
            turretSim.update(0.002);

            turretSimState.setRawRotorPosition(
                turretSim.getAngularPosition().times(ShooterConstants.TURRET_GEAR_REDUCTION));
            turretSimState.setRotorVelocity(
                turretSim.getAngularVelocity().times(ShooterConstants.TURRET_GEAR_REDUCTION));
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
