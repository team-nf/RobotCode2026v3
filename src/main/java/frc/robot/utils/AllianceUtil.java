package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;
import frc.robot.constants.PoseConstants;

/** Utility for robust alliance-aware field target selection. */
public final class AllianceUtil {
  private static boolean lastKnownBlueAlliance = true;

  private AllianceUtil() {}

  /** Refresh cached alliance from Driver Station; call at startup/mode init, not every loop. */
  public static void refreshAllianceFromDriverStation() {
    if(Robot.isReal()) {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      lastKnownBlueAlliance = false;
    } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      lastKnownBlueAlliance = true;
    }
    }
    else
    {
      DriverStation.getAlliance()
        .ifPresent(
            alliance -> {
              if (alliance == DriverStation.Alliance.Red) {
                lastKnownBlueAlliance = false;
              } else if (alliance == DriverStation.Alliance.Blue) {
                lastKnownBlueAlliance = true;
              }
            });
    }
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
