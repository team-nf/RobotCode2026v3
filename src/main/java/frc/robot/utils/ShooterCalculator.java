// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SimConstants;
import frc.robot.constants.TheMachineConstants;

public final class ShooterCalculator {

    private ShooterCalculator() {}

    // Pre-cached unit conversions to avoid .in() calls every loop
    private static final double MIN_FLYWHEEL_RPS = ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond);
    private static final double MAX_FLYWHEEL_RPS = ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond);
    private static final double REST_FLYWHEEL_RPS = ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);
    private static final double MIN_HOOD_DEG = ShooterConstants.MIN_HOOD_ANGLE.in(Degrees);
    private static final double MAX_HOOD_DEG = ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);
    private static final double PASS_HOOD_DEG = ShooterConstants.PASS_HOOD_ANGLE.in(Degrees);

    public static Translation2d getHubTranslation() {
        return AllianceUtil.getHubAimPose().getTranslation();
    }

    private static final double SHOOTER_AXIS_X = TheMachineConstants.SHOOTER_ROTATION_AXIS.getX();
    private static final double SHOOTER_AXIS_Y = TheMachineConstants.SHOOTER_ROTATION_AXIS.getY();

    public static Pose2d getShooterPoseFromRobotPose(Pose2d robotPose) {
        return new Pose2d(
            getShooterXFromRobotPose(robotPose),
            getShooterYFromRobotPose(robotPose),
            robotPose.getRotation());
    }

    public static double getShooterXFromRobotPose(Pose2d robotPose) {
        return getShooterXFromRobotState(robotPose.getX(), robotPose.getRotation().getRadians());
    }

    public static double getShooterYFromRobotPose(Pose2d robotPose) {
        return getShooterYFromRobotState(robotPose.getY(), robotPose.getRotation().getRadians());
    }

    public static double getShooterXFromRobotState(double robotX, double robotHeadingRadians) {
        return robotX + SHOOTER_AXIS_X * Math.cos(robotHeadingRadians) - SHOOTER_AXIS_Y * Math.sin(robotHeadingRadians);
    }

    public static double getShooterYFromRobotState(double robotY, double robotHeadingRadians) {
        return robotY + SHOOTER_AXIS_X * Math.sin(robotHeadingRadians) + SHOOTER_AXIS_Y * Math.cos(robotHeadingRadians);
    }

    public static Translation2d getShooterTranslation(Pose2d robotPose) {
        return getShooterPoseFromRobotPose(robotPose).getTranslation();
    }

    public static double getDistanceToHub(Pose2d pose) {
        tempHubAimPose = AllianceUtil.getHubAimPose();
        tempShooterX = getShooterXFromRobotPose(pose);
        tempShooterY = getShooterYFromRobotPose(pose);
        tempDx = tempHubAimPose.getX() - tempShooterX;
        tempDy = tempHubAimPose.getY() - tempShooterY;
        return Math.hypot(tempDx, tempDy);
    }

    // Reusable result array to avoid allocation every loop
    private static final double[] shootingParams = new double[2];
    private static Pose2d tempHubAimPose;
    private static double tempHubX;
    private static double tempHubY;
    private static double tempShooterX;
    private static double tempShooterY;
    private static double tempDx;
    private static double tempDy;
    private static double tempDistance;
    private static double tempWheelSpeed;
    private static double tempHoodAngleDeg;
    private static double tempY;

    private static final double HOOD_A = 1.79;
    private static final double HOOD_B = 14.43;
    private static final double HOOD_OFFSET = 14.312;
    private static final double HOOD_SHORT_RANGE_BREAKPOINT_METERS = 2.05;
    private static final double HOOD_SHORT_RANGE_DEGREES = 3.5;

    private static final double FLYWHEEL_A = -29.72883;
    private static final double FLYWHEEL_B = 473.27393;
    private static final double FLYWHEEL_C = -2886.63609;
    private static final double FLYWHEEL_D = 8444.7507;
    private static final double FLYWHEEL_E = -11605.2592;
    private static final double FLYWHEEL_F = 7475.15141;
    private static final double FLYWHEEL_CLOSE_RANGE_BREAKPOINT_METERS = 1.7;
    private static final double FLYWHEEL_CLOSE_RANGE_RPM = 1500.0;
    private static final double FLYWHEEL_FAR_RANGE_BREAKPOINT_METERS = 5.0;
    private static final double FLYWHEEL_FAR_RANGE_BASE_RPM = 2631.0;
    private static final double FLYWHEEL_FAR_RANGE_SLOPE_RPM_PER_METER = 20.0;

    private static final double FLYWHEEL_SIM_A = -0.236999;
    private static final double FLYWHEEL_SIM_B = 3.9759;
    private static final double FLYWHEEL_SIM_C = 13.86351;
    private static final double FLYWHEEL_SIM_CLOSE_RANGE_BREAKPOINT_METERS = 1.0;
    private static final double FLYWHEEL_SIM_CLOSE_RANGE_RPS = 17.5;

    private static final double PASS_RPS_A = 250.0;
    private static final double PASS_RPS_B = 1500.0;

    private static final double FLIGHT_TIME_A = -0.0374327;
    private static final double FLIGHT_TIME_B = 0.418028;
    private static final double FLIGHT_TIME_C = 0.277584;
    private static final double FLIGHT_TIME_SHORT_RANGE_BREAKPOINT_METERS = 1.0;
    private static final double FLIGHT_TIME_SHORT_RANGE_SECONDS = 0.65;

    public static double getDistanceToHubWithSpeedCalculation(double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
        tempShooterX = getShooterXFromRobotPose(pose);
        tempShooterY = getShooterYFromRobotPose(pose);
        return getDistanceToHubWithSpeedCalculation(filteredSpeedX, filteredSpeedY, tempShooterX, tempShooterY, time);
    }

    public static double getDistanceToHubWithSpeedCalculation(double filteredSpeedX, double filteredSpeedY, double shooterX, double shooterY, double time) {
        tempHubAimPose = AllianceUtil.getHubAimPose();
        tempHubX = tempHubAimPose.getX();
        tempHubY = tempHubAimPose.getY();
        tempDx = tempHubX - filteredSpeedX * time - shooterX;
        tempDy = tempHubY - filteredSpeedY * time - shooterY;
        return Math.hypot(tempDx, tempDy);
    }

    public static double getXDistanceToHub(Pose2d pose) {
        return Math.abs(AllianceUtil.getHubAimPose().getX() - getShooterXFromRobotPose(pose));
    }

    public static double getXDistanceToHub(double robotX, double robotHeadingRadians) {
        return Math.abs(AllianceUtil.getHubAimPose().getX() - getShooterXFromRobotState(robotX, robotHeadingRadians));
    }

    public static double[] calculateShootingParameters(double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
        tempDistance = getDistanceToHubWithSpeedCalculation(filteredSpeedX, filteredSpeedY, pose, time);
        return calculateShootingParametersFromDistance(tempDistance);
    }

    public static double[] calculateShootingParameters(double filteredSpeedX, double filteredSpeedY, double shooterX, double shooterY, double time) {
        tempDistance = getDistanceToHubWithSpeedCalculation(filteredSpeedX, filteredSpeedY, shooterX, shooterY, time);
        return calculateShootingParametersFromDistance(tempDistance);
    }

    private static double[] calculateShootingParametersFromDistance(double distanceMeters) {
        tempDistance = distanceMeters;

        if (Robot.isReal()) {
            tempWheelSpeed = flywheelRPSFormula(tempDistance) / ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;
        }
        else {
            tempWheelSpeed = flywheelRPSFormulaSIM(tempDistance) / SimConstants.SIMULATION_VELOCITY_TRANSFER_COEFFICIENT;
        }

        tempWheelSpeed = Math.max(
            MIN_FLYWHEEL_RPS,
            Math.min(tempWheelSpeed, MAX_FLYWHEEL_RPS)
        );

        tempHoodAngleDeg = hoodAngleFormula(tempDistance);
        tempHoodAngleDeg = Math.max(MIN_HOOD_DEG, Math.min(tempHoodAngleDeg, MAX_HOOD_DEG));
        shootingParams[0] = tempWheelSpeed;
        shootingParams[1] = tempHoodAngleDeg;
        return shootingParams;
    }

    public static double calculatePassSpeedFromCurrentPose(Pose2d pose) {
        tempWheelSpeed = passRPSFormula(getXDistanceToHub(pose)) / ShooterConstants.SHOOTER_VELOCITY_TRANSFER_COEFFICIENT;
        tempWheelSpeed = Math.max(
            MIN_FLYWHEEL_RPS,
            Math.min(tempWheelSpeed, MAX_FLYWHEEL_RPS)
        );
        return tempWheelSpeed;
    }

    public static double calculatePassSpeedFromCurrentState(double robotX, double robotHeadingRadians) {
        tempWheelSpeed = passRPSFormula(getXDistanceToHub(robotX, robotHeadingRadians));
        tempWheelSpeed = Math.max(
            MIN_FLYWHEEL_RPS,
            Math.min(tempWheelSpeed, MAX_FLYWHEEL_RPS)
        );
        return tempWheelSpeed;
    }

    public static double calculateRestFlywheelSpeed() {
        return REST_FLYWHEEL_RPS;
    }

    public static double calculateRestHoodAngle() {
        return MIN_HOOD_DEG;
    }

    public static double calculatePassHoodAngle() {
        return PASS_HOOD_DEG;
    }

    public static double hoodAngleFormula(double x) {
        if (x < HOOD_SHORT_RANGE_BREAKPOINT_METERS) {
            return HOOD_SHORT_RANGE_DEGREES;
        } else {
            return HOOD_A * x + HOOD_B - HOOD_OFFSET;
        }
    }

    public static double flywheelRPSFormula(double x) {
        if (x < FLYWHEEL_CLOSE_RANGE_BREAKPOINT_METERS) {
            return FLYWHEEL_CLOSE_RANGE_RPM / 60.0;
        }

        tempY = (((((FLYWHEEL_A * x + FLYWHEEL_B) * x + FLYWHEEL_C) * x + FLYWHEEL_D) * x + FLYWHEEL_E) * x + FLYWHEEL_F);

        if (x > FLYWHEEL_FAR_RANGE_BREAKPOINT_METERS) {
            tempY = FLYWHEEL_FAR_RANGE_BASE_RPM;
            tempY += FLYWHEEL_FAR_RANGE_SLOPE_RPM_PER_METER * (x - FLYWHEEL_FAR_RANGE_BREAKPOINT_METERS);
        }

        return tempY / 60.0;
    }

    public static double flywheelRPSFormulaSIM(double x) {
        if (x < FLYWHEEL_SIM_CLOSE_RANGE_BREAKPOINT_METERS) {
            return FLYWHEEL_SIM_CLOSE_RANGE_RPS;
        }

        tempY = ((FLYWHEEL_SIM_A * x + FLYWHEEL_SIM_B) * x + FLYWHEEL_SIM_C);

        return tempY;
    }

    public static double passRPSFormula(double x) {
        tempY = PASS_RPS_A * x + PASS_RPS_B;
        return tempY / 60.0;
    }

    public static double flightTimeOfFuelFormula(double x) {
        if (x < FLIGHT_TIME_SHORT_RANGE_BREAKPOINT_METERS) {
            return FLIGHT_TIME_SHORT_RANGE_SECONDS;
        }

        return ((FLIGHT_TIME_A * x + FLIGHT_TIME_B) * x + FLIGHT_TIME_C) + 0.0;
    }
}
