// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class DriveConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    public static final PathConstraints PATH_CONSTRAINTS_TO_POSE = new PathConstraints(
        3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    public static final PathConstraints PATH_CONSTRAINTS_FOLLOW_PATH = new PathConstraints(
        5.0, 6.0, 3 * Math.PI, 6 * Math.PI);

    public static final AngularVelocity AIMING_MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.7);
    public static final double AIMING_kP = 8.5;
    public static final double AIMING_kI = 30.0;
    public static final double AIMING_kD = 0.5;

    public static final double AIMED_DRIVING_kP = 17.5;
    public static final double AIMED_DRIVING_kI = 40.0;
    public static final double AIMED_DRIVING_KD = 0.0;

    public static final double AIMING_TOLERANCE_RADIANS = Degrees.of(2).in(Radians);
}
