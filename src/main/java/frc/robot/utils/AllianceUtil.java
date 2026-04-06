package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.PoseConstants;

/** Utility for robust alliance-aware field target selection. */
public final class AllianceUtil {
    private static boolean lastKnownBlueAlliance = true;

    private AllianceUtil() {}

    /** Refresh cached alliance from Driver Station; call at startup/mode init, not every loop. */
    public static void refreshAllianceFromDriverStation() {
        DriverStation.getAlliance().ifPresentOrElse(
            alliance -> lastKnownBlueAlliance = alliance == DriverStation.Alliance.Blue,
            () -> {
                if (Container.isBlue != null) {
                    lastKnownBlueAlliance = Container.isBlue;
                }
            }
        );

        Container.isBlue = lastKnownBlueAlliance;
    }

    public static boolean isBlueAlliance() {
        if (Container.isBlue != null) {
            lastKnownBlueAlliance = Container.isBlue;
        }
        return lastKnownBlueAlliance;
    }

    public static Pose2d getHubAimPose() {
        return isBlueAlliance() ? PoseConstants.BLUE_HUB_AIM_POSE : PoseConstants.RED_HUB_AIM_POSE;
    }
}
