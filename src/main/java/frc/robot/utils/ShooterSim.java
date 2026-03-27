package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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

    public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> supplier) {
        this.chassisSpeedsSupplier = supplier;
    }

    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = supplier;
    }

    public void setShouldShootSupplier(Supplier<Boolean> supplier) {
        this.shouldShootSupplier = supplier;
    }

    private double fuelPerSecondLimit = 6.0; // Default limit
    private double lastShootTime = 0;
    private int fuelShotCount = 0;

    public void setFuelPerSecondLimit(double limit) {
        this.fuelPerSecondLimit = limit;
    }

    public void launchFuel() {
        HopperSim hopperSim = HopperSim.getInstance();
        FuelSim fuelSim = FuelSim.getInstance();
        double shooterRPS = shooterRPSSupplier.get();
        double hoodAngle = hoodAngleSupplier.get();

        double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double timeSinceLastShoot = currentTime - lastShootTime;

        if (timeSinceLastShoot >= 1.0 / fuelPerSecondLimit && hopperSim.getCurrentHopperLoad() > 0) {
            Transform3d robotPoseTransform = 
                    new Transform3d(new Translation3d(robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), 0), 
                    new Rotation3d(robotPoseSupplier.get().getRotation()));

            double launchAngle = Math.PI / 2 - Math.toRadians((hoodAngle)*360+18);
            double fuelVelocity = shooterRPS * 2 * Math.PI * SimConstants.SIMULATION_VELOCITY_TRANSFER_COEFFICIENT * ShooterConstants.FLYWHEEL_RADIUS.in(Meters);

            Pose3d leftFuelShootPose = new Pose3d(
                Dimensions.LEFT_SHOOTER_POSE.getX() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.cos(hoodAngle*2*Math.PI),
                Dimensions.LEFT_SHOOTER_POSE.getY(),
                Dimensions.LEFT_SHOOTER_POSE.getZ() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.sin(hoodAngle*2*Math.PI),
                new Rotation3d()
            );

            Pose3d rightFuelShootPose = new Pose3d(
                Dimensions.RIGHT_SHOOTER_POSE.getX() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.cos(hoodAngle*2*Math.PI),
                Dimensions.RIGHT_SHOOTER_POSE.getY(),
                Dimensions.RIGHT_SHOOTER_POSE.getZ() + Dimensions.FUEL_SHOOTER_OFFSET.in(Meters) * Math.sin(hoodAngle*2*Math.PI),
                new Rotation3d()
            );

            Pose3d worldLeftFuelShootPose = leftFuelShootPose.transformBy(robotPoseTransform)
                                                .rotateAround(robotPoseTransform.getTranslation(), new Rotation3d(robotPoseSupplier.get().getRotation()));

            Pose3d worldRightFuelShootPose = rightFuelShootPose.transformBy(robotPoseTransform)
                                                .rotateAround(robotPoseTransform.getTranslation(), new Rotation3d(robotPoseSupplier.get().getRotation()));


            double leftFuelVelocity = fuelVelocity*(1.0 + (Math.random() - 0.5) * 0.05); // Adding slight randomness

            Translation3d leftFuelVelocityVector = new Translation3d(
                leftFuelVelocity * Math.cos(launchAngle) * Math.cos(robotPoseSupplier.get().getRotation().getRadians()),
                leftFuelVelocity * Math.cos(launchAngle) * Math.sin(robotPoseSupplier.get().getRotation().getRadians()),
                leftFuelVelocity * Math.sin(launchAngle)
            );

            leftFuelVelocityVector = leftFuelVelocityVector.plus(new Translation3d(
                chassisSpeedsSupplier.get().vxMetersPerSecond,
                chassisSpeedsSupplier.get().vyMetersPerSecond,
                0
            ));

            hopperSim.removeFuelFromHopper();
            fuelSim.spawnFuel(worldLeftFuelShootPose.getTranslation(), leftFuelVelocityVector);
            fuelShotCount++;

            if (hopperSim.getCurrentHopperLoad() > 0) {

                double rightFuelVelocity = fuelVelocity*(1.0 + (Math.random() - 0.5) * 0.05); // Adding slight randomness

                Translation3d rightFuelVelocityVector = new Translation3d(
                    rightFuelVelocity * Math.cos(launchAngle) * Math.cos(robotPoseSupplier.get().getRotation().getRadians()),
                    rightFuelVelocity * Math.cos(launchAngle) * Math.sin(robotPoseSupplier.get().getRotation().getRadians()),
                    rightFuelVelocity * Math.sin(launchAngle)
                );

                rightFuelVelocityVector = rightFuelVelocityVector.plus(new Translation3d(
                    chassisSpeedsSupplier.get().vxMetersPerSecond,
                    chassisSpeedsSupplier.get().vyMetersPerSecond,
                    0
                ));

                hopperSim.removeFuelFromHopper();
                fuelSim.spawnFuel(worldRightFuelShootPose.getTranslation(), rightFuelVelocityVector);
                fuelShotCount++;
            }

            lastShootTime = currentTime; // Update the last shoot time
        }
    }

    public void updateSim() {
        if (shouldShootSupplier.get()) {
            launchFuel();
        }
    }


}
