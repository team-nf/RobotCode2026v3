// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.EnumSet;
import java.util.function.Supplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TheMachineConstants;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ShooterCalculator;

/**
 * Shooter mechanism subsystem.
 *
 * <p>Owns flywheel, hood, and turret motor control; exposes high-level actions used by
 * {@code TheMachine}; publishes telemetry; and maintains simulation behavior.
 */
public class ShooterSubsystem extends SubsystemBase {

    // Runtime dashboard override used for pit/debug operation.
    private final BooleanTopic manualOverrideTopic = NetworkTableInstance.getDefault()
        .getBooleanTopic("Conf/TheMachine/ManualOverrideEnabled");
    private final BooleanEntry manualOverrideEntry = manualOverrideTopic.getEntry(false);
    private boolean manualOverrideEnabled = false;
    @SuppressWarnings("unused")
    private final int manualOverrideListenerHandle;

    // Flywheel motors: motor1 leader + motor2 follower
    private TalonFX flywheelMotor1;
    private TalonFX flywheelMotor2;

    private TalonFX hoodMotor;
    private TalonFX turretMotor;
    private DutyCycleEncoder turretAbsoluteEncoder;

    private final VelocityVoltage flywheelVelocityControl;
    private final PositionVoltage hoodPositionControl;
    private final PositionVoltage turretPositionControl;

    private final StatusSignal<?> flywheel1VelocitySignal;
    private final StatusSignal<?> flywheel1CurrentSignal;
    private final StatusSignal<?> flywheel1VoltageSignal;
    private final StatusSignal<?> flywheel2VelocitySignal;
    private final StatusSignal<?> flywheel2CurrentSignal;
    private final StatusSignal<?> flywheel2VoltageSignal;
    private final StatusSignal<?> hoodPositionSignal;
    private final StatusSignal<?> hoodVelocitySignal;
    private final StatusSignal<?> hoodCurrentSignal;
    private final StatusSignal<?> hoodVoltageSignal;
    private final StatusSignal<?> turretPositionSignal;
    private final StatusSignal<?> turretVelocitySignal;
    private final StatusSignal<?> turretCurrentSignal;
    private final StatusSignal<?> turretVoltageSignal;

    private static final double MIN_HOOD_DEG = ShooterConstants.MIN_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    private static final double MAX_HOOD_DEG = ShooterConstants.MAX_HOOD_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    private static final double MIN_TURRET_DEG = ShooterConstants.MIN_TURRET_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    private static final double MAX_TURRET_DEG = ShooterConstants.MAX_TURRET_ANGLE.in(edu.wpi.first.units.Units.Degrees);
    private static final double TURRET_SYSID_LIMIT_MARGIN_DEG = 8.0;

    private double flywheelGoalVelocity;
    private double hoodGoalAngle;
    private double turretGoalAngleDegrees;
    private double flywheelTestRPM;
    private double hoodTestAngle;
    private double turretTestAngleDegrees;

    // Reused temp variables to avoid recreating locals in hot paths.
    private double tempTurretDeg;
    private double tempMotorVelocity;
    private double tempRequestedTurretFrameDeg;
    private double tempWrappedTargetDeg;
    private double tempCurrentAngleDeg;
    private double tempAngleErrorDeg;
    private int tempTurretPidSlot;
    private double tempMotorPosition;
    private double tempWrappedNormDeg;
    private double tempRequestedClampedDeg;
    private double tempDeltaDeg;
    private double tempNearestEquivalentDeg;
    private double tempAbsoluteEncoderRot;
    private double tempAbsoluteTurretDeg;
    private double tempTurretMotorRot;
    private double tempTurretAngleClamped;
    private double tempFlywheelRps;
    private double tempFlywheelRpmError;
    private double tempHoodDegError;
    private double tempTurretDegError;
    private double tempHoodAngleDeg;
    private double tempTurretAngleDeg;

    private final VoltageOut sysIdVoltageControl;
    private final SysIdRoutine flywheelSysIdRoutine;
    private final SysIdRoutine hoodSysIdRoutine;
    private final SysIdRoutine turretSysIdRoutine;

    public ShooterSubsystem(Supplier<Double> robotYawSupplier) {

        // NOTE: robotYawSupplier is intentionally kept in constructor signature for compatibility
        // with existing wiring, even though current shooter logic does not consume it directly.

        manualOverrideEntry.setDefault(false);
        manualOverrideEnabled = manualOverrideEntry.get(false);
        manualOverrideListenerHandle = NetworkTableInstance.getDefault().addListener(
            manualOverrideTopic,
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            event -> manualOverrideEnabled = manualOverrideEntry.get(false)
        );

        flywheelMotor1 = new TalonFX(ShooterConstants.FIRST_SHOOTER_MOTOR_ID);
        flywheelMotor2 = new TalonFX(ShooterConstants.SECOND_SHOOTER_MOTOR_ID);
        hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
        turretMotor = new TalonFX(ShooterConstants.TURRET_MOTOR_ID);
        turretAbsoluteEncoder = new DutyCycleEncoder(ShooterConstants.TURRET_THROUGHBORE_DIO_CHANNEL);

        applyConfig(flywheelMotor1, ShooterConstants.SHOOTER_CONFIG, "Flywheel Motor 1 (Leader)");
        applyConfig(flywheelMotor2, ShooterConstants.SHOOTER_CONFIG, "Flywheel Motor 2 (Follower)");
        flywheelMotor2.setControl(new Follower(flywheelMotor1.getDeviceID(), MotorAlignmentValue.Opposed));

        applyConfig(hoodMotor, ShooterConstants.HOOD_CONFIG, "Hood Motor");
        applyConfig(turretMotor, ShooterConstants.TURRET_CONFIG, "Turret Motor");

        flywheelVelocityControl = ShooterConstants.SHOOTER_VELOCITY_CONTROL.clone();
        hoodPositionControl = ShooterConstants.HOOD_POSITION_CONTROL.clone();
        turretPositionControl = ShooterConstants.TURRET_POSITION_CONTROL.clone();
        sysIdVoltageControl = new VoltageOut(0).withEnableFOC(false);

        flywheel1VelocitySignal = flywheelMotor1.getVelocity(false);
        flywheel1CurrentSignal = flywheelMotor1.getStatorCurrent(false);
        flywheel1VoltageSignal = flywheelMotor1.getMotorVoltage(false);

        flywheel2VelocitySignal = flywheelMotor2.getVelocity(false);
        flywheel2CurrentSignal = flywheelMotor2.getStatorCurrent(false);
        flywheel2VoltageSignal = flywheelMotor2.getMotorVoltage(false);

        hoodPositionSignal = hoodMotor.getPosition(false);
        hoodVelocitySignal = hoodMotor.getVelocity(false);
        hoodCurrentSignal = hoodMotor.getStatorCurrent(false);
        hoodVoltageSignal = hoodMotor.getMotorVoltage(false);

        turretPositionSignal = turretMotor.getPosition(false);
        turretVelocitySignal = turretMotor.getVelocity(false);
        turretCurrentSignal = turretMotor.getStatorCurrent(false);
        turretVoltageSignal = turretMotor.getMotorVoltage(false);

        flywheelSysIdRoutine = createSysIdRoutine("shooter/Flywheel", flywheelMotor1, 4.0);
        hoodSysIdRoutine = createSysIdRoutine("shooter/Hood", hoodMotor, 3.0);
        turretSysIdRoutine = createSysIdRoutine("shooter/Turret", turretMotor, 2.0);

        // On startup, align integrated turret motor position with absolute encoder reading.
        syncTurretMotorToAbsoluteEncoder();

        hoodMotor.setPosition(0);
    }

    private SysIdRoutine createSysIdRoutine(String logPrefix, TalonFX motor, double dynamicStepVolts) {
        return new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(dynamicStepVolts),
                null,
                state -> SignalLogger.writeString(logPrefix + "_State", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                output -> {
                    motor.setControl(sysIdVoltageControl.withOutput(output.in(Volts)));
                    SignalLogger.writeDouble(logPrefix + "_Voltage", motor.getMotorVoltage().getValueAsDouble());
                    SignalLogger.writeDouble(logPrefix + "_Position", motor.getPosition().getValueAsDouble());
                    SignalLogger.writeDouble(logPrefix + "_Velocity", motor.getVelocity().getValueAsDouble());
                },
                null,
                this
            )
        );
    }

    private boolean isTurretNearSysIdLimit() {
        tempTurretDeg = getTurretAngleDegreesRaw();
        return tempTurretDeg <= (MIN_TURRET_DEG + TURRET_SYSID_LIMIT_MARGIN_DEG)
            || tempTurretDeg >= (MAX_TURRET_DEG - TURRET_SYSID_LIMIT_MARGIN_DEG);
    }

    public Command flywheelSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.quasistatic(direction).finallyDo(interrupted -> stopFlywheel());
    }

    public Command flywheelSysIdDynamic(SysIdRoutine.Direction direction) {
        return flywheelSysIdRoutine.dynamic(direction).finallyDo(interrupted -> stopFlywheel());
    }

    public Command hoodSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return hoodSysIdRoutine.quasistatic(direction).finallyDo(interrupted -> hoodMotor.set(0.0));
    }

    public Command hoodSysIdDynamic(SysIdRoutine.Direction direction) {
        return hoodSysIdRoutine.dynamic(direction).finallyDo(interrupted -> hoodMotor.set(0.0));
    }

    public Command turretSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return turretSysIdRoutine
            .quasistatic(direction)
            .until(this::isTurretNearSysIdLimit)
            .finallyDo(interrupted -> turretMotor.set(0.0));
    }

    public Command turretSysIdDynamic(SysIdRoutine.Direction direction) {
        return turretSysIdRoutine
            .dynamic(direction)
            .until(this::isTurretNearSysIdLimit)
            .finallyDo(interrupted -> turretMotor.set(0.0));
    }

    private void applyConfig(TalonFX motor, com.ctre.phoenix6.configs.TalonFXConfiguration config, String name) {
        // Retry configuration a few times to handle transient CAN startup timing.
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
        // Convert mechanism-side RPS to motor-side RPS through gear ratio.
        tempMotorVelocity = velocityRPS * ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
        flywheelMotor1.setControl(flywheelVelocityControl.withVelocity(tempMotorVelocity));
    }

    private void setHoodAngle(double angleInputDegrees) {
        // All callers provide desired hood angle in DEGREES.
        hoodGoalAngle = Math.max(MIN_HOOD_DEG, Math.min(angleInputDegrees, MAX_HOOD_DEG));
        hoodMotor.setControl(hoodPositionControl.withPosition(hoodGoalAngle / 360.0 * ShooterConstants.HOOD_GEAR_REDUCTION));
    }

    public double setTurretAngleDegrees(double requestedAngleDegrees) {
        // Public turret API uses robot-frame degrees. Convert to turret-mechanism frame first.
        tempRequestedTurretFrameDeg = robotFrameToTurretFrameDeg(requestedAngleDegrees);

        // Resolve to nearest legal equivalent angle to avoid long wraps.
        tempWrappedTargetDeg = wrapTurretTargetToCurrent(tempRequestedTurretFrameDeg);
        tempCurrentAngleDeg = getTurretAngleDegreesRaw();
        tempAngleErrorDeg = Math.abs(normalizeToMinus180To180(tempWrappedTargetDeg - tempCurrentAngleDeg));

        // Use more aggressive PID slot near target for tighter final convergence.
        tempTurretPidSlot = tempAngleErrorDeg <= ShooterConstants.TURRET_SMALL_ERROR_THRESHOLD_DEG
            ? ShooterConstants.TURRET_AGGRESSIVE_SLOT
            : ShooterConstants.TURRET_GENTLE_SLOT;

        tempMotorPosition = tempWrappedTargetDeg / 360.0 * ShooterConstants.TURRET_GEAR_REDUCTION;
        turretMotor.setControl(
            turretPositionControl
                .withSlot(tempTurretPidSlot)
                .withPosition(tempMotorPosition)
        );
    return turretFrameToRobotFrameDeg(tempWrappedTargetDeg);
    }

    private double normalizeToMinus180To180(double angleDeg) {
        tempWrappedNormDeg = angleDeg % 360.0;
        if (tempWrappedNormDeg > 180.0) {
            tempWrappedNormDeg -= 360.0;
        } else if (tempWrappedNormDeg < -180.0) {
            tempWrappedNormDeg += 360.0;
        }
        return tempWrappedNormDeg;
    }

    private double clampTurretRange(double angleDeg) {
        return Math.max(MIN_TURRET_DEG, Math.min(angleDeg, MAX_TURRET_DEG));
    }

    private double turretFrameToRobotFrameDeg(double turretFrameDeg) {
        return normalizeToMinus180To180(turretFrameDeg + ShooterConstants.TURRET_ZERO_IN_ROBOT_FRAME_DEG);
    }

    private double robotFrameToTurretFrameDeg(double robotFrameDeg) {
        return normalizeToMinus180To180(robotFrameDeg - ShooterConstants.TURRET_ZERO_IN_ROBOT_FRAME_DEG);
    }

    private double wrapTurretTargetToCurrent(double requestedAngleDegrees) {
        tempRequestedClampedDeg = clampTurretRange(requestedAngleDegrees);
        tempCurrentAngleDeg = getTurretAngleDegreesRaw();

        // Select nearest angular equivalent relative to current angle.
        tempDeltaDeg = normalizeToMinus180To180(tempRequestedClampedDeg - tempCurrentAngleDeg);
        tempNearestEquivalentDeg = tempCurrentAngleDeg + tempDeltaDeg;

        // Re-wrap if nearest equivalent would violate soft limits.
        if (tempNearestEquivalentDeg < MIN_TURRET_DEG) {
            tempNearestEquivalentDeg += 360.0;
        } else if (tempNearestEquivalentDeg > MAX_TURRET_DEG) {
            tempNearestEquivalentDeg -= 360.0;
        }

        return clampTurretRange(tempNearestEquivalentDeg);
    }

    private void stopFlywheel() {
        flywheelMotor1.set(0.0);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
            flywheel1VelocitySignal,
            //flywheel1CurrentSignal,
            //flywheel1VoltageSignal,
            flywheel2VelocitySignal,
            //flywheel2CurrentSignal,
            //flywheel2VoltageSignal,
            hoodPositionSignal,
            //hoodVelocitySignal,
            //hoodCurrentSignal,
            //hoodVoltageSignal,
            turretPositionSignal
            //turretVelocitySignal,
            //turretCurrentSignal,
            //turretVoltageSignal
            );
    }

    public void syncTurretMotorToAbsoluteEncoder() {
        // Read through-bore absolute encoder and seed integrated motor position to match.
        tempAbsoluteEncoderRot = turretAbsoluteEncoder.get();
        // Absolute encoder is interpreted in turret frame (mechanism-local angle).
        tempAbsoluteTurretDeg = tempAbsoluteEncoderRot
                * ShooterConstants.TURRET_ABSOLUTE_DEGREES_PER_ENCODER_ROTATION
                + ShooterConstants.TURRET_ABSOLUTE_OFFSET_DEGREES;
        tempAbsoluteTurretDeg = clampTurretRange(tempAbsoluteTurretDeg);

        tempTurretMotorRot = tempAbsoluteTurretDeg / 360.0 * ShooterConstants.TURRET_GEAR_REDUCTION;
        turretMotor.setPosition(tempTurretMotorRot);
        turretGoalAngleDegrees = turretFrameToRobotFrameDeg(tempAbsoluteTurretDeg);
    }

    // ===== Action Methods =====

    /** Stop all motors. */
    public void zero() {
        flywheelGoalVelocity = 0;
        hoodGoalAngle = ShooterCalculator.calculateRestHoodAngle();
        turretGoalAngleDegrees = 180;
        stopFlywheel();
        setHoodAngle(hoodGoalAngle);
        turretGoalAngleDegrees = setTurretAngleDegrees(turretGoalAngleDegrees);
    }

    /** Hold flywheel at idle speed, park hood at min angle. */
    public void rest() {
        flywheelGoalVelocity = ShooterCalculator.calculateRestFlywheelSpeed();
        hoodGoalAngle = ShooterCalculator.calculateRestHoodAngle();
        setFlywheelSpeed(flywheelGoalVelocity);
        setHoodAngle(hoodGoalAngle);
    }

    /** Set flywheel speed + hood angle + turret angle (degrees). */
    public void shoot(double velocityRPS, double hoodAngle, double turretAngleDegrees) {

        if(!manualOverrideEnabled)
        {
            // Normal auto-setpoint path from aiming commands.
            flywheelGoalVelocity = velocityRPS;
            hoodGoalAngle = hoodAngle;
            turretGoalAngleDegrees = setTurretAngleDegrees(turretAngleDegrees);
            setFlywheelSpeed(flywheelGoalVelocity);
            setHoodAngle(hoodGoalAngle);
            return;
        }
        else
        {
            // Pit/debug fallback constants when manual override is enabled.
            flywheelGoalVelocity = 25;
            hoodGoalAngle = 0.0;
            turretGoalAngleDegrees = setTurretAngleDegrees(180);
            setFlywheelSpeed(flywheelGoalVelocity);
            setHoodAngle(hoodGoalAngle);
            return;
        }
        
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void shoot(double velocityRPS, double hoodAngle) {
        shoot(velocityRPS, hoodAngle, 0.0);
    }

    /** Set flywheel speed + hood angle + turret angle (degrees) for pass. */
    public void pass(double velocityRPS, double hoodAngle, double turretAngleDegrees) {
        shoot(velocityRPS, hoodAngle, turretAngleDegrees);
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void pass(double velocityRPS, double hoodAngle) {
        pass(velocityRPS, hoodAngle, 0.0);
    }

    /** Direct manual control for testing. */
    public void test(double velocityRPS, double hoodAngle, double turretAngleDegrees) {
        shoot(velocityRPS, hoodAngle, turretAngleDegrees);
    }

    /** Backward-compatible overload that keeps turret at 0 deg. */
    public void test(double velocityRPS, double hoodAngle) {
        test(velocityRPS, hoodAngle, 0.0);
    }

    /** Direct manual control using stored test values. */
    public void test() {
        test(flywheelTestRPM, hoodTestAngle, turretTestAngleDegrees);
    }

    /** Get the current flywheel velocity in RPS (mechanism side). */
    public double getFlywheel1Velocity() {
        return flywheel1VelocitySignal.getValueAsDouble() / ShooterConstants.FLYWHEEL_GEAR_REDUCTION;
    }

    /** Get the current flywheel goal velocity in RPS (mechanism side). */
    public double getFlywheelGoalVelocityRps() {
        return flywheelGoalVelocity;
    }

    public double getFlywheel1SpeedAbs() {
        return Math.abs(getFlywheel1Velocity());
    }

    /** Get the current hood position in rotations (mechanism side). */
    public double getHoodPosition() {
        return hoodPositionSignal.getValueAsDouble() / ShooterConstants.HOOD_GEAR_REDUCTION;
    }
    

    public double getTurretAngleDegrees() {
        return turretFrameToRobotFrameDeg(getTurretAngleDegreesRaw());
    }

    private double getTurretAngleDegreesRaw() {
        return turretPositionSignal.getValueAsDouble() / ShooterConstants.TURRET_GEAR_REDUCTION * 360.0;
    }

    public double getTurretAngleRadians()
    {
        return Math.toRadians(getTurretAngleDegrees());
    }

    public double getHoodAngleDegrees() {
        return getHoodPosition() * 360.0;
    }

    // Pre-cached allowable error thresholds
    private static final double FLYWHEEL_ERROR_RPS = ShooterConstants.FLYWHEEL_ALLOWABLE_ERROR.in(RotationsPerSecond);
    private static final double HOOD_ERROR_DEG = ShooterConstants.HOOD_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);
    private static final double TURRET_ERROR_DEG = ShooterConstants.TURRET_ALLOWABLE_ERROR.in(edu.wpi.first.units.Units.Degrees);

    /** Check if the flywheel is at the target speed within allowable error. */
    public boolean isFlywheelAtSpeed() {
        return Math.abs(getFlywheel1SpeedAbs() - Math.abs(flywheelGoalVelocity)) < FLYWHEEL_ERROR_RPS;
    }

    /** Check if the hood is at the target angle within allowable error. */
    public boolean isHoodAtAngle() {
        return Math.abs(getHoodAngleDegrees() - hoodGoalAngle) < HOOD_ERROR_DEG;
    }

    public boolean isTurretAtAngle() {
        // Require both position error and low angular velocity before declaring ready.
        return Math.abs(normalizeToMinus180To180(getTurretAngleDegrees() - turretGoalAngleDegrees)) < TURRET_ERROR_DEG
            && Math.abs(turretVelocitySignal.getValueAsDouble()) < ShooterConstants.TURRET_ALLOWABLE_SPEED_TO_SHOOT.in(RotationsPerSecond);
    }

    /** Check if the shooter is ready to fire (flywheel + hood + turret). */
    public boolean isReadyToShoot() {
        return isFlywheelAtSpeed() && isHoodAtAngle() && isTurretAtAngle() && getFlywheel1SpeedAbs() > 15;
    }

    public boolean isManualOverrideEnabled() {
        return manualOverrideEnabled;
    }

    public boolean isHoodClosed() {
        return getHoodPosition() <  0.01;
    }

    public void publishTelemetry() {
        // Keep dashboard-reported turret angle bounded to legal mechanism range.
        tempTurretAngleDeg = getTurretAngleDegrees();
        tempHoodAngleDeg = getHoodAngleDegrees();
        tempTurretAngleClamped = Math.max(MIN_TURRET_DEG, Math.min(tempTurretAngleDeg, MAX_TURRET_DEG));
        tempFlywheelRps = getFlywheel1SpeedAbs();
        tempFlywheelRpmError = (Math.abs(flywheelGoalVelocity) - tempFlywheelRps) * 60.0;
        tempHoodDegError = hoodGoalAngle - tempHoodAngleDeg;
        tempTurretDegError = normalizeToMinus180To180(turretGoalAngleDegrees - tempTurretAngleDeg);

    SmartDashboard.putNumber("Shooter/Flywheel1VelocityRps", TelemetryConstants.roundTelemetry(flywheel1VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/Flywheel1CurrentA", TelemetryConstants.roundTelemetry(flywheel1CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/Flywheel1VoltageV", TelemetryConstants.roundTelemetry(flywheel1VoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Shooter/Flywheel2VelocityRps", TelemetryConstants.roundTelemetry(flywheel2VelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/Flywheel2CurrentA", TelemetryConstants.roundTelemetry(flywheel2CurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/Flywheel2VoltageV", TelemetryConstants.roundTelemetry(flywheel2VoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Shooter/HoodMotorPositionRot", TelemetryConstants.roundTelemetry(hoodPositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/HoodMotorVelocityRps", TelemetryConstants.roundTelemetry(hoodVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/HoodMotorCurrentA", TelemetryConstants.roundTelemetry(hoodCurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/HoodMotorVoltageV", TelemetryConstants.roundTelemetry(hoodVoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Shooter/TurretMotorPositionRot", TelemetryConstants.roundTelemetry(turretPositionSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/TurretMotorVelocityRps", TelemetryConstants.roundTelemetry(turretVelocitySignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/TurretMotorCurrentA", TelemetryConstants.roundTelemetry(turretCurrentSignal.getValueAsDouble()));
    SmartDashboard.putNumber("Shooter/TurretMotorVoltageV", TelemetryConstants.roundTelemetry(turretVoltageSignal.getValueAsDouble()));

    SmartDashboard.putNumber("Shooter/FlywheelRPS", TelemetryConstants.roundTelemetry(tempFlywheelRps));
    SmartDashboard.putNumber("Shooter/HoodAngleRot", TelemetryConstants.roundTelemetry(getHoodPosition()));
    SmartDashboard.putNumber("Shooter/HoodAngleDeg", TelemetryConstants.roundTelemetry(tempHoodAngleDeg));
    SmartDashboard.putNumber("Shooter/HoodGoalDeg", TelemetryConstants.roundTelemetry(hoodGoalAngle));
    SmartDashboard.putNumber("Shooter/TurretAngleDeg", TelemetryConstants.roundTelemetry(tempTurretAngleClamped));
    SmartDashboard.putNumber("Shooter/TurretGoalDeg", TelemetryConstants.roundTelemetry(turretGoalAngleDegrees));
    SmartDashboard.putNumber("Shooter/FlywheelRpmError", TelemetryConstants.roundTelemetry(tempFlywheelRpmError));
    SmartDashboard.putNumber("Shooter/HoodDegError", TelemetryConstants.roundTelemetry(tempHoodDegError));
    SmartDashboard.putNumber("Shooter/TurretDegError", TelemetryConstants.roundTelemetry(tempTurretDegError));
        SmartDashboard.putBoolean("Shooter/FlywheelReady", isFlywheelAtSpeed());
        SmartDashboard.putBoolean("Shooter/HoodReady", isHoodAtAngle());
        SmartDashboard.putBoolean("Shooter/TurretReady", isTurretAtAngle());
        SmartDashboard.putBoolean("Shooter/Ready", isReadyToShoot());
        SmartDashboard.putBoolean("Shooter/ManualOverrideEnabled", manualOverrideEnabled);
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
    private TalonFXSimState fw1SimState;
    private TalonFXSimState fw2SimState;
    private TalonFXSimState hoodSimState;
    private TalonFXSimState turretSimState;

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
            // Step each plant model and mirror resulting rotor states into Phoenix sim objects.
            // --- Flywheel sim update ---
            fw1SimState = flywheelMotor1.getSimState();
            fw2SimState = flywheelMotor2.getSimState();

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
            hoodSimState = hoodMotor.getSimState();

            hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

            hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
            hoodSim.update(0.002);

            hoodSimState.setRawRotorPosition(
                Rotations.of(hoodSim.getAngleRads() / (2 * Math.PI) * ShooterConstants.HOOD_GEAR_REDUCTION));
            hoodSimState.setRotorVelocity(
                RadiansPerSecond.of(hoodSim.getVelocityRadPerSec()).times(ShooterConstants.HOOD_GEAR_REDUCTION));

            // --- Turret sim update ---
            turretSimState = turretMotor.getSimState();

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

    double turretLLX = TheMachineConstants.TURRET_LL_POSE.getX() + TheMachineConstants.SHOOTER_ROTATION_AXIS.getX();
    double turretLLY = TheMachineConstants.TURRET_LL_POSE.getY() + TheMachineConstants.SHOOTER_ROTATION_AXIS.getY();
    double turretLLZ = TheMachineConstants.TURRET_LL_POSE.getZ() + TheMachineConstants.SHOOTER_ROTATION_AXIS.getZ();
    double turretLLPitchDegrees = Math.toDegrees(TheMachineConstants.TURRET_LL_POSE.getRotation().getY());
    double turretLLHeading = 0;

    @Override
    public void periodic() {
        // Refresh cached signals once per loop before any dependent reads.
        refreshStatusSignals();

        // Recompute turret-mounted Limelight pose as turret rotates around shooter axis.
        // getTurretAngleRadians() is opposite of the robot-space convention expected by this
        // transform, so negate it to keep camera pose movement aligned with turret direction.
        turretLLHeading = -getTurretAngleRadians();
        turretLLX = TheMachineConstants.TURRET_LL_POSE.getX() * Math.cos(turretLLHeading) - TheMachineConstants.TURRET_LL_POSE.getY() * Math.sin(turretLLHeading);
        turretLLY = TheMachineConstants.TURRET_LL_POSE.getY() * Math.cos(turretLLHeading) + TheMachineConstants.TURRET_LL_POSE.getX() * Math.sin(turretLLHeading);

        turretLLX = turretLLX + TheMachineConstants.SHOOTER_ROTATION_AXIS.getX();
        turretLLY = turretLLY + TheMachineConstants.SHOOTER_ROTATION_AXIS.getY();

    SmartDashboard.putNumber("Throughbore", TelemetryConstants.roundTelemetry(turretAbsoluteEncoder.get()));
    SmartDashboard.putNumber("TurretLLHeading", TelemetryConstants.roundTelemetry(Math.toDegrees(turretLLHeading)));

                                                                
        // Push updated camera pose to Limelight on real robot only.
        if(Robot.isReal()) 
            LimelightHelpers.setCameraPose_RobotSpace("limelight-turret", turretLLX, turretLLY, turretLLZ, 0, 
                                                                                                                turretLLPitchDegrees, 
                                                                                                                 -Math.toDegrees(turretLLHeading));
    }
}
 