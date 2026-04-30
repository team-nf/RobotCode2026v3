// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.Robot;

/** Add your docs here. */
public class TelemetryConstants {

  // Set to false to disable all DataLog recording (e.g. during practice to save storage).
  public static final boolean SHOULD_LOG = false;

  public static final int TELEMETRY_DECIMAL_PLACES = 2; // tweak as needed
  public static final double TELEMETRY_ROUNDING_FACTOR = Math.pow(10.0, TELEMETRY_DECIMAL_PLACES);

  /** Round a value to the configured telemetry decimal places. */
  public static double roundTelemetry(double value) {
    return Math.round(value * TELEMETRY_ROUNDING_FACTOR) / TELEMETRY_ROUNDING_FACTOR;
  }
}
