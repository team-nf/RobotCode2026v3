package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Dimensions;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SimConstants;
import java.util.function.Supplier;

/** Add your docs here. */
public class ShooterSim {

  private Supplier<Double> shooterRPSSupplier;
  private Supplier<Double> hoodAngleSupplier;
  private Supplier<Double> turretAngleSupplier;
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private Supplier<Pose2d> robotPoseSupplier;
  private Supplier<Boolean> shouldShootSupplier;

  private static ShooterSim instance;

  public static ShooterSim getInstance() {
    if (instance == null) {
      instance = new ShooterSim();
    }
    return instance;
  }

  public void setShooterRPSSupplier(Supplier<Double> supplier) {
    this.shooterRPSSupplier = supplier;
  }

  public void setHoodAngleSupplier(Supplier<Double> supplier) {
    this.hoodAngleSupplier = supplier;
  }

  public void setTurretAngleSupplier(Supplier<Double> supplier) {
    this.turretAngleSupplier = supplier;
  }

  public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> supplier) {
    this.chassisSpeedsSupplier = supplier;
  }

  public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
    this.robotPoseSupplier = supplier;
  }

  public void setShouldShootSupplier(Supplier<Boolean> supplier) {
    this.shouldShootSupplier = supplier;
  }

  private double fuelPerSecondLimit = 8.0; // Default limit
  private double lastShootTime = 0;

  private static final Transform3d SHOOTER_TRANSFORM_FROM_ROBOT =
      new Transform3d(Dimensions.SHOOTER_POSE.getTranslation(), new Rotation3d());

  public void setFuelPerSecondLimit(double limit) {
    this.fuelPerSecondLimit = limit;
  }

  public void launchFuel() {
    HopperSim hopperSim = HopperSim.getInstance();
    FuelSim fuelSim = FuelSim.getInstance();

    Pose2d robotPose = robotPoseSupplier.get();
    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
    double shooterRPS = shooterRPSSupplier.get();
    double hoodAngle = hoodAngleSupplier.get();
    double turretAngleDeg = turretAngleSupplier.get();

    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double timeSinceLastShoot = currentTime - lastShootTime;

    if (timeSinceLastShoot >= 1.0 / fuelPerSecondLimit && hopperSim.getCurrentHopperLoad() > 0) {
      Pose3d worldRobotPose = new Pose3d(robotPose);

      double launchAngle = Math.PI / 2 - Math.toRadians((hoodAngle) * 360 + 18);
      double worldYaw = robotPose.getRotation().getRadians() + Math.toRadians(turretAngleDeg);
      double fuelVelocity =
          shooterRPS
              * 2
              * Math.PI
              * SimConstants.SIMULATION_VELOCITY_TRANSFER_COEFFICIENT
              * ShooterConstants.FLYWHEEL_RADIUS.in(Meters);

      Pose3d worldFuelShootPose = worldRobotPose.transformBy(SHOOTER_TRANSFORM_FROM_ROBOT);

      double randomizedFuelVelocity = fuelVelocity * (1.0 + (Math.random() - 0.5) * 0.05);

      Translation2d shooterOffsetField =
          new Translation2d(Dimensions.SHOOTER_POSE.getX(), Dimensions.SHOOTER_POSE.getY())
              .rotateBy(robotPose.getRotation());

      double shooterVx =
          chassisSpeeds.vxMetersPerSecond
              - chassisSpeeds.omegaRadiansPerSecond * shooterOffsetField.getY();
      double shooterVy =
          chassisSpeeds.vyMetersPerSecond
              + chassisSpeeds.omegaRadiansPerSecond * shooterOffsetField.getX();

      Translation3d fuelVelocityVector =
          new Translation3d(
              randomizedFuelVelocity * Math.cos(launchAngle) * Math.cos(worldYaw),
              randomizedFuelVelocity * Math.cos(launchAngle) * Math.sin(worldYaw),
              randomizedFuelVelocity * Math.sin(launchAngle));

      fuelVelocityVector = fuelVelocityVector.plus(new Translation3d(shooterVx, shooterVy, 0));

      hopperSim.removeFuelFromHopper();
      fuelSim.spawnFuel(worldFuelShootPose.getTranslation(), fuelVelocityVector);

      lastShootTime = currentTime; // Update the last shoot time
    }
  }

  public void updateSim() {
    if (shouldShootSupplier.get()) {
      launchFuel();
    }
  }
}
