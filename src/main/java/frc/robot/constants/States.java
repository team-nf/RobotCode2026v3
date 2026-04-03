package frc.robot.constants;

/**
 * Shared enums describing high-level robot operating states.
 */
public class States {
    public static class SwerveStates {
        /** Drivetrain operating modes. */
        public enum SwerveState {
            TELEOP,
            AIMING,
            PATH_FOLLOWING,
        }
    }

    public static class TheMachineStates {
        /** High-level coordinated mechanism states (intake/shooter/cargo path). */
        public enum TheMachineState {
            ZERO,
            IDLE_RETRACTED,
            IDLE_DEPLOYED,
            INTAKE,
            SHOOT,
            REVERSE,
            TEST,
            NONE,
            IDLE,
            GET_READY,
            PASS,
            GET_READY_PASS,
        }
    }

}
