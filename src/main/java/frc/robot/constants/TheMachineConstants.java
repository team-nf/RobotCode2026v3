package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class TheMachineConstants {

    // Addressable LED strip hardware configuration.
    public static int LED_PWM_PORT = 0;
    public static int LED_STRIP_LENGTH = 19;

    public static final double INTAKE_EXTENSION_ANGLE_DEGREES = -9.8;

    public static Pose3d SHOOTER_ZERO_POSE = new Pose3d(-0.1475, 0.1475, 0.3865, new Rotation3d(0, 0, 0));
    public static Pose3d HOOD_RETRACTED_POSE = new Pose3d(-0.0425, 0.1475, 0.4895, new Rotation3d(0, 0, 0));
    public static Pose3d INTAKE_RETRACTED_POSE = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0, 0, 0));
    
    public static Pose3d SHOOTER_ROTATION_AXIS = new Pose3d(-0.1475, 0.1475, 0.3635, new Rotation3d(0, 0, 0));

    public static Pose3d TURRET_LL_POSE = new Pose3d(0.1783, 0.0, 0.0765, new Rotation3d(0, Math.toRadians(20), 0));

    public static int DRIVER_CONTROLLER_PORT_ID = 0;
    public static int EMERGENCY_CONTROLLER_PORT_ID = 1;
}
