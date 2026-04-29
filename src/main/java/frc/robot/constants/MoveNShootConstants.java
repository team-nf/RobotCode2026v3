// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

public class MoveNShootConstants {

  // Turret lookahead: how far ahead (seconds) to predict robot position for setpoint calculation
  public static final double TURRET_LOOKAHEAD_SEC = 0.1;

  // Shoot (teleop) drive limits
  public static final double SHOOT_MAX_SPEED_FRACTION = 0.3;
  public static final double SHOOT_TRANSLATION_SLEW_RATE = 2.0;
  public static final double SHOOT_MAX_ANGULAR_RATE_RPS = 0.30;
  public static final int SHOOT_FILTER_SIZE = 4;

  // Pass (teleop) drive limits
  public static final double PASS_MAX_SPEED_FRACTION = 0.4;
  public static final double PASS_MAX_ANGULAR_RATE_RPS = 0.4;
  public static final int PASS_FILTER_SIZE = 4;

  // Setpoint change deadbands — only re-send if the new value differs by more than this amount.
  // Keeps CAN traffic low while still tracking meaningful changes.
  public static final double SETPOINT_RPS_DEADBAND = 0.5;
  public static final double SETPOINT_HOOD_DEG_DEADBAND = 0.5;
  public static final double SETPOINT_TURRET_DEG_DEADBAND = 1.0;
}
