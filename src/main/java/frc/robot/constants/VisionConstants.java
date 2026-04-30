package frc.robot.constants;

public class VisionConstants {

  // Camera names as configured in the Limelight web UI.
  public static final String LL_TURRET_NAME = "limelight-turret";
  public static final String LL_FIXED_NAME = "limelight-normal";

  // --- Rejection thresholds ---

  // Reject all vision updates when the gyro is spinning faster than this (deg/s).
  // Fast rotation makes heading-locked MegaTag2 unreliable.
  public static final double MAX_GYRO_RATE_DEG_PER_SEC = 360.0;

  // Reject turret camera updates when the LL's onboard IMU reports a Z acceleration spike.
  // Indicates a hard collision that may have shifted the camera mount.
  public static final double MAX_ACCEL_Z_FOR_VISION_G = 120.0;

  // MT1 only: reject estimates where the average tag distance exceeds this (meters).
  // MT1 single-tag solve degrades quickly at range; MT2 handles distance better.
  public static final double MT1_MAX_TAG_DIST_METERS = 3.5;

  // MT1 only: per-tag ambiguity threshold (0–1). Values near 1.0 mean two pose solutions
  // are nearly equally valid — the solver is essentially guessing. Reject the whole estimate.
  public static final double MAX_TAG_AMBIGUITY = 0.9;

  // Maximum tag distance allowed when hard-resetting pose from MT1 (meters).
  public static final double MT1_RESET_MAX_TAG_DIST_METERS = 3.0;

  // --- Confidence-scaled stdDev tuning ---

  // Base XY position stdDev (meters) before distance/tag-count scaling.
  // Final stdDev = base * avgTagDist² / tagCount.
  // Tune these to balance how aggressively vision corrects odometry:
  //   lower = vision trusted more at a given distance/tag-count.
  public static final double BASE_XY_STD_DEV_MT2 = 0.5;
  public static final double BASE_XY_STD_DEV_MT2_SHOOT = 0.2;
  public static final double BASE_XY_STD_DEV_MT1 = 0.5;

  // Floor on computed XY stdDev so no measurement ever gets infinite weight,
  // even with many tags at very close range.
  public static final double MIN_XY_STD_DEV = 0.1;

  // Heading is always ignored in vision updates (gyro is more accurate).
  public static final double HEADING_STD_DEV_IGNORED = 9999999.0;
}
