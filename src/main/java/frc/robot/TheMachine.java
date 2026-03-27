package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.States.TheMachineStates.TheMachineState;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TheMachine {

    private final ShooterSubsystem shooterSubsystem;
    private final HopperSubsystem hopperSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final FeederSubsystem feederSubsystem;

    private TheMachineState state = TheMachineState.IDLE_RETRACTED;

    private boolean intakeWithOffset = false;
    
    public TheMachine(ShooterSubsystem shooterSubsystem, HopperSubsystem hopperSubsystem, IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopperSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.feederSubsystem = feederSubsystem;
    }

    public void zero() {
        shooterSubsystem.zero();
        feederSubsystem.zero();
        hopperSubsystem.zero();
        intakeSubsystem.close();
        state = TheMachineState.ZERO;
    }

    public void idleRetracted() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.close();
        state = TheMachineState.IDLE_RETRACTED;
    }

    public void idleDeployed() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.deploy();
        state = TheMachineState.IDLE_DEPLOYED;
    }

    public void idle() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        intakeSubsystem.idleBetween();
        state = TheMachineState.IDLE;
    }

    public void intake() {
        shooterSubsystem.rest();
        feederSubsystem.reverse();
        hopperSubsystem.zero();
        if (intakeWithOffset) {
            intakeSubsystem.intakeWithOffset();
        } else {
            intakeSubsystem.intake();
        }
        state = TheMachineState.INTAKE;
    }

    public void getReady(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shooterSubsystem.shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.deploy();
        state = TheMachineState.GET_READY;
    }

    public void getReady(double velocityRPS, double hoodAngleRotations) {
        getReady(velocityRPS, hoodAngleRotations, 0.0);
    }

    public void shoot(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shooterSubsystem.shoot(velocityRPS, hoodAngleRotations, turretAngleDegrees);
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.SHOOT;
    }

    public void shoot(double velocityRPS, double hoodAngleRotations) {
        shoot(velocityRPS, hoodAngleRotations, 0.0);
    }

    public void getReadyPass(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shooterSubsystem.pass(velocityRPS, hoodAngleRotations, turretAngleDegrees);
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.deploy();
        state = TheMachineState.GET_READY_PASS;
    }

    public void getReadyPass(double velocityRPS, double hoodAngleRotations) {
        getReadyPass(velocityRPS, hoodAngleRotations, 0.0);
    }

    public void pass(double velocityRPS, double hoodAngleRotations, double turretAngleDegrees) {
        shooterSubsystem.pass(velocityRPS, hoodAngleRotations, turretAngleDegrees);
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.PASS;
    }

    public void pass(double velocityRPS, double hoodAngleRotations) {
        pass(velocityRPS, hoodAngleRotations, 0.0);
    }

    public void reverse() {
        shooterSubsystem.zero();
        feederSubsystem.reverse();
        hopperSubsystem.reverse();
        intakeSubsystem.reverse();
        state = TheMachineState.REVERSE;
    }

    public void test() {
        shooterSubsystem.test();
        feederSubsystem.feed();
        hopperSubsystem.feed();
        intakeSubsystem.feed();
        state = TheMachineState.TEST;
    }

    public void changeIntakeMode() {
        intakeWithOffset = !intakeWithOffset;
    }

    public boolean isIntakeWithOffset() {
        return intakeWithOffset;
    }

    public boolean isState(TheMachineState state) {
        return this.state == state;
    }

    public boolean isShooterReady()
    {
        return shooterSubsystem.isReadyToShoot();
    }

  private StructPublisher<Pose3d> intakePosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/IntakePose", Pose3d.struct).publish();

  private StructPublisher<Pose3d> shooterPosePublisher = NetworkTableInstance.getDefault()
    .getStructTopic("Sim/ShooterPose", Pose3d.struct).publish();

   private StructPublisher<Pose3d> hoodPosePublisher = NetworkTableInstance.getDefault()
  .getStructTopic("Sim/HoodPose", Pose3d.struct).publish();



  public void calculateSubsytemPoses() {
    double intakeExtensionMeters = intakeSubsystem.getIntakeExtensionMeters();
    double intakeExtensionAngleRad = Math.toRadians(TheMachineConstants.INTAKE_EXTENSION_ANGLE_DEGREES);
    double intakeExtensionX = intakeExtensionMeters * Math.cos(intakeExtensionAngleRad);
    double intakeExtensionZ = intakeExtensionMeters * Math.sin(intakeExtensionAngleRad);

    SmartDashboard.putNumber("IntakeExtensionMeters", intakeExtensionMeters);
        SmartDashboard.putNumber("IntakeExtensionAngleDeg", TheMachineConstants.INTAKE_EXTENSION_ANGLE_DEGREES);

    Pose3d intakePose = TheMachineConstants.INTAKE_RETRACTED_POSE
                                                    .plus(new Transform3d(intakeExtensionX, 0, intakeExtensionZ, new Rotation3d(0, 0, 0)));


    double turretYawRad = Math.toRadians(shooterSubsystem.getTurretAngleDegrees());
    double hoodPitchRad = shooterSubsystem.getHoodPosition() * 2.0 * Math.PI;
    Pose3d shooterZeroPose = TheMachineConstants.SHOOTER_ZERO_POSE;
    Pose3d hoodZeroPose = TheMachineConstants.HOOD_RETRACTED_POSE;

    Pose3d shooterPose = new Pose3d(
        shooterZeroPose.getTranslation(),
        shooterZeroPose.getRotation().plus(new Rotation3d(0, 0, turretYawRad))
    );

    Transform3d shooterToHoodAtZero = new Transform3d(shooterZeroPose, hoodZeroPose);
    Pose3d hoodYawPose = shooterPose.plus(shooterToHoodAtZero);
    Pose3d hoodPose = hoodYawPose.rotateAround(
        hoodYawPose.getTranslation(),
        new Rotation3d(hoodPitchRad, 0, 0)
    );

    SmartDashboard.putNumber("Shooter/TurretPoseDeg", Math.toDegrees(turretYawRad));
    SmartDashboard.putNumber("Shooter/HoodPoseDeg", Math.toDegrees(hoodPitchRad));

    intakePosePublisher.set(intakePose);
    shooterPosePublisher.set(shooterPose);
    hoodPosePublisher.set(hoodPose);

  }

  public void publishTelemetry() {
    shooterSubsystem.publishTelemetry();
    feederSubsystem.publishTelemetry();
    hopperSubsystem.publishTelemetry();
    intakeSubsystem.publishTelemetry();
  }

  public SubsystemBase[] getSubsystems() {
    return new SubsystemBase[] {shooterSubsystem, feederSubsystem, hopperSubsystem, intakeSubsystem};
  }
}
