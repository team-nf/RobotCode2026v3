package frc.robot.constants;

public class States {
        public static class SwerveStates {
            public enum SwerveState {
                TELEOP,
                AIMING,
                PATH_FOLLOWING,
            }
    }

        public static class TheMachineStates {
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
