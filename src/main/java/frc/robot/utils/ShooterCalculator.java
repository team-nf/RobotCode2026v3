// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TheMachineConstants;

public final class ShooterCalculator {

  private ShooterCalculator() {}

  // Pre-cached unit conversions to avoid .in() calls every loop
  private static final double MIN_FLYWHEEL_RPS =
      ShooterConstants.MIN_FLYWHEEL_SPEED.in(RotationsPerSecond);
  private static final double MAX_FLYWHEEL_RPS =
      ShooterConstants.MAX_FLYWHEEL_SPEED.in(RotationsPerSecond);
  private static final double REST_FLYWHEEL_RPS =
      ShooterConstants.FLYWHEEL_REST_SPEED.in(RotationsPerSecond);
  private static final double MIN_HOOD_DEG = ShooterConstants.MIN_HOOD_ANGLE.in(Degrees);
  private static final double MAX_HOOD_DEG = ShooterConstants.MAX_HOOD_ANGLE.in(Degrees);

  // Lookup tables sampled every 0.5 meters from 0.5m to 7.0m.
  // Index 0 -> 0.5m, index 1 -> 1.0m, ..., index 13 -> 7.0m.
  private static final double LOOKUP_MIN_DISTANCE_METERS = 0.5;
  private static final double LOOKUP_STEP_METERS = 0.5;
  private static final double LOOKUP_MAX_DISTANCE_METERS = 6.5;

   // Columns: [0] = shooter RPS, [1] = hood angle deg
  private static final double[][] SHOOTER_LOOKUP_TABLE_PREV = {
    {38, 4}, // 0.5 meters
    {29, 4.5}, // 1.0 meters
    {30.5, 5}, // 1.5 meters
    {31.6, 5.2}, // 2.0 meters
    {34, 5.4}, // 2.5 meters
    {35.8, 5.6}, // 3.0 meters
    {38, 8}, // 3.5 meters
    {39, 9}, // 4.0 meters
    {41, 10}, // 4.5 meters
    {42.6, 12}, // 5.0 meters
    {45, 18}, // 5.5 meters
    {47.7, 20.0}, // 6.0 meters
    {50.5, 20} // 6.5 meters
  };

  // Columns: [0] = shooter RPS, [1] = hood angle deg, [2] = flight time sec
  private static final double[][] SHOOTER_LOOKUP_TABLE = {
    {28,   0,    0.65}, // 0.5 meters
    {28,   0,    0.71}, // 1.0 meters
    {28, 3,    1}, // 1.5 meters
    {29.8, 5,  1}, // 2.0 meters
    {31, 5,  1}, // 2.5 meters
    {33,   6.5,    1.5}, // 3.0 meters
    {35, 7.5,    1.6}, // 3.5 meters
    {36.2,   10,    1.8}, // 4.0 meters
    {38,   12,   2}, // 4.5 meters
    {39,   14,   2.1}, // 5.0 meters
    {39.5,   16, 2.1}, // 5.5 meters
    {39.5, 17, 2.1}, // 6.0 meters
    {41.5, 17, 2.3}, // 6.5 meters
  };

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
    return robotX
        + SHOOTER_AXIS_X * Math.cos(robotHeadingRadians)
        - SHOOTER_AXIS_Y * Math.sin(robotHeadingRadians);
  }

  public static double getShooterYFromRobotState(double robotY, double robotHeadingRadians) {
    return robotY
        + SHOOTER_AXIS_X * Math.sin(robotHeadingRadians)
        + SHOOTER_AXIS_Y * Math.cos(robotHeadingRadians);
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

  // Reusable result arrays to avoid allocation every loop
  private static final double[] shootingParams = new double[2];
  private static final double[] passParams = new double[2];
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
  private static double tempClampedDistance;
  private static double tempLookupIndex;
  private static int tempLowIndex;
  private static int tempHighIndex;
  private static double tempInterpolationT;

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

  // Pass lookup table: columns are [RPS, hood angle deg, flight time sec]
  // Index 0 = 0.0 m, index 1 = 0.5 m, ..., index 24 = 12.0 m (0.5 m steps)
  // Flight times seeded from flightTimeOfFuelFormula * 0.8; beyond 6 m extrapolated linearly.
  // Tune all three columns on the field.
  private static final double[][] PASS_LOOKUP_TABLE = {
    {30, 0, 1}, // 1.0 meters
    {30, 2.5, 1}, // 2.0 meters
    {32.5, 14.0, 1.00}, // 3.0 meters
    {36, 16.0, 1.6}, // 4.0 meters
    {40, 16.0, 1.7}, // 5.0 meters
    {43.0, 16.0, 1.9}, // 6.0 meters — formula unreliable past here; tune on field
    {46, 16.0, 2.1}, // 7.0 meters
    {49, 17.0, 2.3}, // 8.0 meters
    {53, 20.0, 2.5}, // 9.0 meters
    {56, 20.0, 2.6}, // 10.0 meters
    {59, 20.0, 2.7}, // 11.0 meters
    {63, 20.0, 2.8}, // 12.0 meters
    {66, 20, 2.9}, // 13.0 meters
    {69, 20.0, 3}, // 14.0 meters
    {72, 20.0, 3.1}, // 15.0 meters
    {75, 20.0, 3.2}  // 16.0 meters

  };
  private static final double PASS_LOOKUP_MIN_DISTANCE_METERS = 1.0;
  private static final double PASS_LOOKUP_MAX_DISTANCE_METERS = 16.0;
  private static final double LOOKUP_STEP_METERS_PASS = 1.0;


  private static final double FLIGHT_TIME_A = -0.0374327;
  private static final double FLIGHT_TIME_B = 0.418028;
  private static final double FLIGHT_TIME_C = 0.277584;
  private static final double FLIGHT_TIME_SHORT_RANGE_BREAKPOINT_METERS = 1.0;
  private static final double FLIGHT_TIME_SHORT_RANGE_SECONDS = 0.65;

  public static double getDistanceToHubWithSpeedCalculation(
      double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
    tempShooterX = getShooterXFromRobotPose(pose);
    tempShooterY = getShooterYFromRobotPose(pose);
    return getDistanceToHubWithSpeedCalculation(
        filteredSpeedX, filteredSpeedY, tempShooterX, tempShooterY, time);
  }

  public static double getDistanceToHubWithSpeedCalculation(
      double filteredSpeedX, double filteredSpeedY, double shooterX, double shooterY, double time) {
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
    return Math.abs(
        AllianceUtil.getHubAimPose().getX()
            - getShooterXFromRobotState(robotX, robotHeadingRadians));
  }

  public static double[] calculateShootingParameters(
      double filteredSpeedX, double filteredSpeedY, Pose2d pose, double time) {
    tempDistance = getDistanceToHubWithSpeedCalculation(filteredSpeedX, filteredSpeedY, pose, time);
    return calculateShootingParametersFromDistance(tempDistance);
  }

  public static double[] calculateShootingParameters(
      double filteredSpeedX, double filteredSpeedY, double shooterX, double shooterY, double time) {
    tempDistance =
        getDistanceToHubWithSpeedCalculation(
            filteredSpeedX, filteredSpeedY, shooterX, shooterY, time);
    return calculateShootingParametersFromDistance(tempDistance);
  }

  private static double[] calculateShootingParametersFromDistance(double distanceMeters) {
    tempDistance = distanceMeters;

    if (Robot.isReal()) {
      tempWheelSpeed = getShooterRpsFromLookupTable(tempDistance);
      tempHoodAngleDeg = getShooterHoodDegFromLookupTable(tempDistance);
    } else {
      tempWheelSpeed = flywheelRPSFormulaSIM(tempDistance);
      tempHoodAngleDeg = hoodAngleFormula(tempDistance);
    }

    tempWheelSpeed = Math.max(MIN_FLYWHEEL_RPS, Math.min(tempWheelSpeed, MAX_FLYWHEEL_RPS));

    tempHoodAngleDeg = Math.max(MIN_HOOD_DEG, Math.min(tempHoodAngleDeg, MAX_HOOD_DEG));
    shootingParams[0] = tempWheelSpeed * ShooterConstants.RPS_LOOKUP_COEFFICIENT;
    shootingParams[1] = tempHoodAngleDeg;
    return shootingParams;
  }

  /**
   * Pass ballistics — no speed compensation (auto variant).
   * Distance is full 2D from shooter to the selected pass aim pose.
   */
  public static double[] calculatePassParameters(
      double shooterX, double shooterY, Pose2d passAimPose) {
    tempDx = passAimPose.getX() - shooterX;
    tempDy = passAimPose.getY() - shooterY;
    tempDistance = Math.hypot(tempDx, tempDy);
    return calculatePassParametersFromDistance(tempDistance);
  }

  /**
   * Pass ballistics — speed-compensated (teleop variant).
   * Adjusts the effective aim point by robot velocity × flight time, then computes
   * the 2D distance from shooter to that adjusted point for lookup interpolation.
   */
  public static double[] calculatePassParameters(
      double filteredSpeedX,
      double filteredSpeedY,
      double shooterX,
      double shooterY,
      Pose2d passAimPose) {
    tempDx = passAimPose.getX() - shooterX;
    tempDy = passAimPose.getY() - shooterY;
    tempDistance = Math.hypot(tempDx, tempDy);
    // Reuse tempDistance as flight time, then overwrite with adjusted distance.
    tempDistance = getPassFlightTime(tempDistance);
    tempDx = passAimPose.getX() - filteredSpeedX * tempDistance - shooterX;
    tempDy = passAimPose.getY() - filteredSpeedY * tempDistance - shooterY;
    tempDistance = Math.hypot(tempDx, tempDy);
    return calculatePassParametersFromDistance(tempDistance);
  }

  private static double[] calculatePassParametersFromDistance(double distanceMeters) {
    passParams[0] = Math.max(MIN_FLYWHEEL_RPS, Math.min(getPassLookupValue(distanceMeters, 0), MAX_FLYWHEEL_RPS));
    passParams[1] = Math.max(MIN_HOOD_DEG, Math.min(getPassLookupValue(distanceMeters, 1), MAX_HOOD_DEG));
    return passParams;
  }

  public static double calculateRestFlywheelSpeed() {
    return REST_FLYWHEEL_RPS;
  }

  public static double calculateRestHoodAngle() {
    return MIN_HOOD_DEG;
  }

  private static double getLookupValue(double distanceMeters, int valueColumn) {
    // Clamp to lookup domain.
    if (distanceMeters <= LOOKUP_MIN_DISTANCE_METERS) {
      return SHOOTER_LOOKUP_TABLE[0][valueColumn];
    }
    if (distanceMeters >= LOOKUP_MAX_DISTANCE_METERS) {
      return SHOOTER_LOOKUP_TABLE[SHOOTER_LOOKUP_TABLE.length - 1][valueColumn];
    }

    // Fractional index in 0.5m bins, then linear interpolation between bins.
    tempClampedDistance =
        Math.max(LOOKUP_MIN_DISTANCE_METERS, Math.min(distanceMeters, LOOKUP_MAX_DISTANCE_METERS));
    tempLookupIndex = (tempClampedDistance - LOOKUP_MIN_DISTANCE_METERS) / LOOKUP_STEP_METERS;
    tempLowIndex = (int) Math.floor(tempLookupIndex);
    tempHighIndex = Math.min(tempLowIndex + 1, SHOOTER_LOOKUP_TABLE.length - 1);
    tempInterpolationT = tempLookupIndex - tempLowIndex;

    return SHOOTER_LOOKUP_TABLE[tempLowIndex][valueColumn]
        + (SHOOTER_LOOKUP_TABLE[tempHighIndex][valueColumn]
                - SHOOTER_LOOKUP_TABLE[tempLowIndex][valueColumn])
            * tempInterpolationT;
  }

  private static double getPassLookupValue(double distanceMeters, int valueColumn) {
    if (distanceMeters <= PASS_LOOKUP_MIN_DISTANCE_METERS) {
      return PASS_LOOKUP_TABLE[0][valueColumn];
    }
    if (distanceMeters >= PASS_LOOKUP_MAX_DISTANCE_METERS) {
      return PASS_LOOKUP_TABLE[PASS_LOOKUP_TABLE.length - 1][valueColumn];
    }
    tempClampedDistance =
        Math.max(PASS_LOOKUP_MIN_DISTANCE_METERS, Math.min(distanceMeters, PASS_LOOKUP_MAX_DISTANCE_METERS));
    tempLookupIndex = (tempClampedDistance - PASS_LOOKUP_MIN_DISTANCE_METERS) / (LOOKUP_STEP_METERS_PASS);
    tempLowIndex = (int) Math.floor(tempLookupIndex);
    tempHighIndex = Math.min(tempLowIndex + 1, PASS_LOOKUP_TABLE.length - 1);
    tempInterpolationT = tempLookupIndex - tempLowIndex;
    return PASS_LOOKUP_TABLE[tempLowIndex][valueColumn]
        + (PASS_LOOKUP_TABLE[tempHighIndex][valueColumn]
                - PASS_LOOKUP_TABLE[tempLowIndex][valueColumn])
            * tempInterpolationT;
  }

  // Name intentionally follows requested spelling.
  public static double getShooterRpsFromLookupTable(double distanceMeters) {
    tempWheelSpeed = getLookupValue(distanceMeters, 0);
    return Math.max(MIN_FLYWHEEL_RPS, Math.min(tempWheelSpeed, MAX_FLYWHEEL_RPS));
  }

  public static double getShooterHoodDegFromLookupTable(double distanceMeters) {
    tempHoodAngleDeg = getLookupValue(distanceMeters, 1);
    return Math.max(MIN_HOOD_DEG, Math.min(tempHoodAngleDeg, MAX_HOOD_DEG));
  }

  /** Shooter flight time (seconds) interpolated from column 2 of SHOOTER_LOOKUP_TABLE. */
  public static double getShooterFlightTime(double distanceMeters) {
    return getLookupValue(distanceMeters, 2) * Container.fTimeCoefficient + Container.loopTimeOffset;
  }

  /** Pass flight time (seconds) interpolated from column 2 of PASS_LOOKUP_TABLE. */
  public static double getPassFlightTime(double distanceMeters) {
    return getPassLookupValue(distanceMeters, 2);
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

    tempY =
        (((((FLYWHEEL_A * x + FLYWHEEL_B) * x + FLYWHEEL_C) * x + FLYWHEEL_D) * x + FLYWHEEL_E) * x
            + FLYWHEEL_F);

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

  public static double flightTimeOfFuelFormula(double x) {
    if (x < FLIGHT_TIME_SHORT_RANGE_BREAKPOINT_METERS) {
      return FLIGHT_TIME_SHORT_RANGE_SECONDS;
    }

    return ((FLIGHT_TIME_A * x + FLIGHT_TIME_B) * x + FLIGHT_TIME_C) + 0.05;
  }
}
