package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Dimensions;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SimConstants;

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

    public void setFuelPerSecondLimit(double limit) {
        this.fuelPerSecondLimit = limit;
    }

    public void launchFuel() {
        HopperSim hopperSim = HopperSim.getInstance();
        FuelSim fuelSim = FuelSim.getInstance();
        double shooterRPS = shooterRPSSupplier.get();
        double hoodAngle = hoodAngleSupplier.get();
        double turretAngleDeg = turretAngleSupplier.get();

        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double timeSinceLastShoot = currentTime - lastShootTime;

        if (timeSinceLastShoot >= 1.0 / fuelPerSecondLimit && hopperSim.getCurrentHopperLoad() > 0) {
            Transform3d robotPoseTransform = 
                    new Transform3d(new Translation3d(robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), 0), 
                    new Rotation3d(robotPoseSupplier.get().getRotation()));

            double launchAngle = Math.PI / 2 - Math.toRadians((hoodAngle)*360+18);
            double worldYaw = robotPoseSupplier.get().getRotation().getRadians() + Math.toRadians(turretAngleDeg);
            double fuelVelocity = shooterRPS * 2 * Math.PI * SimConstants.SIMULATION_VELOCITY_TRANSFER_COEFFICIENT * ShooterConstants.FLYWHEEL_RADIUS.in(Meters);

            Pose3d fuelShootPose = new Pose3d(
                Dimensions.SHOOTER_POSE.getX() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.cos(hoodAngle * 2 * Math.PI),
                Dimensions.SHOOTER_POSE.getY(),
                Dimensions.SHOOTER_POSE.getZ() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.sin(hoodAngle * 2 * Math.PI),
                new Rotation3d()
            );

            Pose3d worldFuelShootPose = fuelShootPose.transformBy(robotPoseTransform);

            double randomizedFuelVelocity = fuelVelocity * (1.0 + (Math.random() - 0.5) * 0.05);

            Translation3d fuelVelocityVector = new Translation3d(
                randomizedFuelVelocity * Math.cos(launchAngle) * Math.cos(worldYaw),
                randomizedFuelVelocity * Math.cos(launchAngle) * Math.sin(worldYaw),
                randomizedFuelVelocity * Math.sin(launchAngle)
            );

            fuelVelocityVector = fuelVelocityVector.plus(new Translation3d(
                chassisSpeedsSupplier.get().vxMetersPerSecond,
                chassisSpeedsSupplier.get().vyMetersPerSecond,
                0
            ));

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
