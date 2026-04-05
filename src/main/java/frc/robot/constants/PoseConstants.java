package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseConstants {
    public static final Pose2d START_POSE_RED_RIGHT = new Pose2d(12.14, 7.63, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d START_POSE_RED_MIDDLE = new Pose2d(13.1, 4.025, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d START_POSE_RED_LEFT = new Pose2d(12.14, 0.425, new Rotation2d(Math.toRadians(180)));


    public static final Pose2d START_POSE_BLUE_RIGHT = new Pose2d(4.405, 0.425, new Rotation2d(Math.toRadians(0)));
    public static final Pose2d START_POSE_BLUE_MIDDLE = new Pose2d(3.5, 4.025, new Rotation2d(Math.toRadians(180)));
    public static final Pose2d START_POSE_BLUE_LEFT = new Pose2d(4.405, 7.625, new Rotation2d(Math.toRadians(0)));


    public static final Pose2d RED_WALL_INTAKE_POSE = new Pose2d(15.1, 2.9, new Rotation2d(Math.toRadians(-31)));
    public static final Pose2d RED_SHOOT_NEAR_TRENCH1_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));
    public static final Pose2d RED_SHOOT_NEAR_TRENCH2_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));

    public static final Pose2d BLUE_WALL_INTAKE_POSE = new Pose2d(2.0, 2.0, new Rotation2d());
    public static final Pose2d BLUE_SHOOT_NEAR_TRENCH1_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));
    public static final Pose2d BLUE_SHOOT_NEAR_TRENCH2_POSE = new Pose2d(13.5, 0.95, new Rotation2d(Math.toRadians(159.305)));


    public static final Pose2d BLUE_HUB_AIM_POSE = new Pose2d(4.61, 4.02, new Rotation2d());
    public static final Pose2d RED_HUB_AIM_POSE = new Pose2d(11.75, 4.02, new Rotation2d());

    public static final Pose2d BLUE_PASS_LEFT = new Pose2d(3.5, 6, new Rotation2d());
    public static final Pose2d BLUE_PASS_RIGHT = new Pose2d(3.5,2, new Rotation2d());

    public static final Pose2d RED_PASS_LEFT = new Pose2d(12.5, 6, new Rotation2d());
    public static final Pose2d RED_PASS_RIGHT = new Pose2d(12.5,2, new Rotation2d());
}
