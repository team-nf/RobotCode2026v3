// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import frc.robot.Robot;

/** Add your docs here. */
public class TelemetryConstants {
    public static boolean SHOULD_SHOOTER_HARDWARE_COMMUNICATE = false;
    public static boolean SHOULD_SHOOTER_CONTROL_COMMUNICATE = false;

    public static boolean SHOULD_FEEDER_HARDWARE_COMMUNICATE = false;
    public static boolean SHOULD_FEEDER_CONTROL_COMMUNICATE = false;

    public static boolean SHOULD_HOPPER_HARDWARE_COMMUNICATE = false;
    public static boolean SHOULD_HOPPER_CONTROL_COMMUNICATE = false;

    public static boolean SHOULD_INTAKE_HARDWARE_COMMUNICATE = false;
    public static boolean SHOULD_INTAKE_CONTROL_COMMUNICATE = false
    ;

    public static boolean SHOULD_SWERVE_DATA_COMMUNICATE = false;
    public static boolean SHOULD_SWERVE_FIELD_COMMUNICATE = false;
    public static boolean SHOULD_SWERVE_CTRE_COMMUNICATE = false;

    public static boolean SHOULD_THEMACHINE_DATA_COMMUNICATE = true;

    public static boolean SHOULD_THEMACHINE_SIM_POSES_COMMUNICATE = Robot.isSimulation();
    public static boolean SHOULD_SCHEDULER_COMMUNICATE = false;

    public static final int TELEMETRY_DECIMAL_PLACES = 2; // tweak as needed
    public static final double TELEMETRY_ROUNDING_FACTOR =
            Math.pow(10.0, TELEMETRY_DECIMAL_PLACES);

    /** Round a value to the configured telemetry decimal places. */
    public static double roundTelemetry(double value) {
        return Math.round(value * TELEMETRY_ROUNDING_FACTOR)
                / TELEMETRY_ROUNDING_FACTOR;
    }
}
