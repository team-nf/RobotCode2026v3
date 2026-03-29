// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.constants.Dimensions;
import frc.robot.constants.PoseConstants;
import frc.robot.constants.ShooterConstants;

public final class ShooterCalculator {

    private ShooterCalculator() {}

    // Pre-cached unit conversions to avoid .in() calls every loop
    private static final double MIN_FLYWHEEL_RPS = ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond);
    private static final double MAX_FLYWHEEL_RPS = ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond);
    private static final double REST_FLYWHEEL_RPS = ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);
    private static final double MIN_HOOD_ROT = ShooterConstants.MIN_HOOD_ANGLE.in(Rotations);
    private static final double MAX_HOOD_DEG = ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);
    private static final double PASS_HOOD_ROT = ShooterConstants.PASS_HOOD_ANGLE.in(Rotations);
    private static final double HOOD_OFFSET_DEG = ShooterConstants.HOOD_ANGLE_OFFSET.in(Degrees);

    public static Translation2d getHubTranslation() {
        return Boolean.TRUE.equals(Container.isBlue)
            ? PoseConstants.BLUE_HUB_AIM_POSE.getTranslation()
            : PoseConstants.RED_HUB_AIM_POSE.getTranslation();
    }

    private static final Transform2d SHOOTER_OFFSET_FROM_ROBOT = new Transform2d(
        Dimensions.SHOOTER_POSE.getX(),
        Dimensions.SHOOTER_POSE.getY(),
        Rotation2d.kZero
    );

    public static Pose2d getShooterPoseFromRobotPose(Pose2d robotPose) {
        return robotPose.transformBy(SHOOTER_OFFSET_FROM_ROBOT);
    }

    public static Translation2d getShooterTranslation(Pose2d robotPose) {
        return getShooterPoseFromRobotPose(robotPose).getTranslation();
    }

    public static double getDistanceToHub(Pose2d pose) {
        return getHubTranslation().getDistance(getShooterTranslation(pose));
    }

    // Reusable result array to avoid allocation every loop
    private static final double[] shootingParams = new double[2];

    public static double getDistanceToHubWithSpeedCalculation(double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
        Translation2d hub = getHubTranslation();
        Translation2d shooterTranslation = getShooterTranslation(pose);
        double dx = hub.getX() - filteredSpeedX * time - shooterTranslation.getX();
        double dy = hub.getY() - filteredSpeedY * time - shooterTranslation.getY();
        return Math.hypot(dx, dy);
    }

    public static double getXDistanceToHub(Pose2d pose) {
        return Math.abs(getHubTranslation().getX() - getShooterTranslation(pose).getX());
    }

    public static double[] calculateShootingParameters(double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
        double distance = getDistanceToHubWithSpeedCalculation(filteredSpeedX, filteredSpeedY, pose, time);

        double wheelSpeed;

        if (Robot.isReal()) {
            wheelSpeed = flywheelRPSFormula(distance) / ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;
        }
        else {
            wheelSpeed = flywheelRPSFormulaSIM(distance) / ShooterConstants.SHOOTER_VELOCITY_SIM_TRANSFER_COEFFICIENT;
        }

        wheelSpeed = Math.max(
            MIN_FLYWHEEL_RPS,
            Math.min(wheelSpeed, MAX_FLYWHEEL_RPS)
        );

        double hoodAngle = hoodAngleFormula(distance);
        double maxDeg = HOOD_OFFSET_DEG + MAX_HOOD_DEG;
        hoodAngle = Math.max(HOOD_OFFSET_DEG, Math.min(hoodAngle, maxDeg));
        hoodAngle = (hoodAngle - HOOD_OFFSET_DEG) / 360;

        shootingParams[0] = wheelSpeed;
        shootingParams[1] = hoodAngle;
        return shootingParams;
    }

    public static double calculatePassSpeedFromCurrentPose(Pose2d pose) {
        double wheelSpeed = passRPSFormula(getXDistanceToHub(pose));
        wheelSpeed = Math.max(
            MIN_FLYWHEEL_RPS,
            Math.min(wheelSpeed, MAX_FLYWHEEL_RPS)
        );
        return wheelSpeed;
    }

    public static double calculateRestFlywheelSpeed() {
        return REST_FLYWHEEL_RPS;
    }

    public static double calculateRestHoodAngle() {
        return MIN_HOOD_ROT;
    }

    public static double calculatePassHoodAngle() {
        return PASS_HOOD_ROT;
    }

    public static double hoodAngleFormula(double x) {
        double a = 1.79;
        double b = 14.43;

        if (x < 2.05) {
            return 18;
        } else {
            return a * x + b;
        }
    }

    public static double flywheelRPSFormula(double x) {
        double a = -29.72883;
        double b = 473.27393;
        double c = -2886.63609;
        double d = 8444.7507;
        double f = -11605.2592;
        double g = 7475.15141;

        if (x < 1.7) {
            return 1500;
        }

        double y = (((((a * x + b) * x + c) * x + d) * x + f) * x + g);

        if (x > 5) {
            y = 2631;
            y += 20 * (x - 5);
        }

        return y / 60;
    }

        public static double flywheelRPSFormulaSIM(double x) {
        double a = -0.236999;
        double b = 3.9759;
        double c = 13.86351;


        if (x < 1) {
            return 17.5;
        }

        double y = ((a * x + b) * x + c);

        return y;
    }

    public static double passRPSFormula(double x) {
        double a = 250;
        double b = 1500;

        double y = a * x + b;

        return y / 60;
    }

    public static double flightTimeOfFuelFormula(double x) {
        double a = -0.0374327;
        double b = 0.418028;
        double c = 0.277584;

        if (x < 1) {
            return 0.65;
        }

        return ((a * x + b) * x + c) + 0.0;
    }
}
