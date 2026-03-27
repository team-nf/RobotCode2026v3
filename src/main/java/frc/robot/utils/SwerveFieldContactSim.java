package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.constants.Dimensions;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SwerveFieldContactSim {
    public final double simLoopTimeSec = 0.001;

    private static final int EDGE_POINTS_PER_SIDE = 8;
    private static final int CORNER_COUNT = 4;
    private static final int TOTAL_EDGE_POINTS = EDGE_POINTS_PER_SIDE * CORNER_COUNT;
    private static final double INTAKE_DEPLOYED_OFFSET = 0.2;

    private static final double HALF_COLLISION_DIST = Dimensions.BUMPER_COLLISION_DISTANCE.in(Meters) / 2.0;
    private static final double COS_45 = Math.cos(Math.PI / 4);
    private static final double SIN_45 = Math.sin(Math.PI / 4);
    private static final double DIAGONAL_OFFSET = HALF_COLLISION_DIST * COS_45;
    private static final double DIAGONAL_LATERAL = HALF_COLLISION_DIST * SIN_45;

    private static SwerveFieldContactSim instance;

    private CommandSwerveDrivetrain m_swerveDrivetrain;
    private Supplier<Boolean> isIntakeDeployed;

    private Pose2d currentSimPose;
    private Pose2d prevSimPose;

    private final Pose2d[] cornerBuffer = new Pose2d[CORNER_COUNT];
    private final Pose2d[] edgeBuffer = new Pose2d[TOTAL_EDGE_POINTS];
    private final boolean[] cornerCollisionFlags = new boolean[CORNER_COUNT];

    private final StructArrayPublisher<Pose2d> cornerPosePublisher =
            NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Swerve/CornerPoses", Pose2d.struct).publish();
    private final StructArrayPublisher<Pose2d> edgePosePublisher =
            NetworkTableInstance.getDefault().getStructArrayTopic("Sim/Swerve/EdgePoses", Pose2d.struct).publish();

    private SwerveFieldContactSim() {}

    public static SwerveFieldContactSim getInstance() {
        if (instance == null) {
            instance = new SwerveFieldContactSim();
        }
        return instance;
    }

    public void setSwerveDrivetrain(CommandSwerveDrivetrain swerveDrivetrain) {
        m_swerveDrivetrain = swerveDrivetrain;
        Pose2d startPose = swerveDrivetrain.getInitialStartPose();
        currentSimPose = startPose;
        prevSimPose = startPose;
    }

    public void setIntakeDeployedSupplier(Supplier<Boolean> isIntakeDeployed) {
        this.isIntakeDeployed = isIntakeDeployed;
    }

    public CommandSwerveDrivetrain getSwerveDrivetrain() {
        return m_swerveDrivetrain;
    }

    public Pose2d getPose() {
        return currentSimPose;
    }

    public void handleSwerveSimFieldCollisions() {
        if (m_swerveDrivetrain == null) return;

        currentSimPose = m_swerveDrivetrain.getState().Pose;

        double frontOffset = (isIntakeDeployed != null && isIntakeDeployed.get()) ? INTAKE_DEPLOYED_OFFSET : 0.0;

        cornerBuffer[0] = currentSimPose.transformBy(new Transform2d( DIAGONAL_OFFSET + frontOffset,  DIAGONAL_LATERAL, Rotation2d.kZero));
        cornerBuffer[1] = currentSimPose.transformBy(new Transform2d( DIAGONAL_OFFSET + frontOffset, -DIAGONAL_LATERAL, Rotation2d.kZero));
        cornerBuffer[2] = currentSimPose.transformBy(new Transform2d(-DIAGONAL_OFFSET,                DIAGONAL_LATERAL, Rotation2d.kZero));
        cornerBuffer[3] = currentSimPose.transformBy(new Transform2d(-DIAGONAL_OFFSET,               -DIAGONAL_LATERAL, Rotation2d.kZero));

        boolean[] results = checkFieldCollisionWithDirections();

        double correctedX = currentSimPose.getX();
        double correctedY = currentSimPose.getY();

        if (results[0] && correctedX < prevSimPose.getX()) correctedX = prevSimPose.getX();
        if (results[1] && correctedX > prevSimPose.getX()) correctedX = prevSimPose.getX();
        if (results[2] && correctedY < prevSimPose.getY()) correctedY = prevSimPose.getY();
        if (results[3] && correctedY > prevSimPose.getY()) correctedY = prevSimPose.getY();

        boolean collisionDetected = false;
        for (boolean b : results) {
            if (b) { collisionDetected = true; break; }
        }

        if (collisionDetected) {
            m_swerveDrivetrain.resetPose(new Pose2d(correctedX, correctedY, prevSimPose.getRotation()));
        } else {
            prevSimPose = currentSimPose;
        }
    }

    private boolean[] checkFieldCollisionWithDirections() {
        boolean xPlusCollided = false;
        boolean xMinusCollided = false;
        boolean yPlusCollided = false;
        boolean yMinusCollided = false;
        boolean ccwCollided = false;
        boolean cwCollided = false;

        for (int i = 0; i < CORNER_COUNT; i++) {
            double x = cornerBuffer[i].getX();
            double y = cornerBuffer[i].getY();

            boolean hitY = checkCollision(y, Dimensions.getCollisionValuesForY(x));
            boolean hitX = checkCollision(x, Dimensions.getCollisionValuesForX(y));

            if (hitY) {
                if (isCloserToLow(y, Dimensions.getCollisionValuesForY(x))) yPlusCollided = true;
                else yMinusCollided = true;
            }
            if (hitX) {
                if (isCloserToLow(x, Dimensions.getCollisionValuesForX(y))) xPlusCollided = true;
                else xMinusCollided = true;
            }

            cornerCollisionFlags[i] = hitX || hitY;
        }

        if (cornerCollisionFlags[0] && !cornerCollisionFlags[1] && !cornerCollisionFlags[2]) ccwCollided = true;
        if (cornerCollisionFlags[1] && !cornerCollisionFlags[0] && !cornerCollisionFlags[3]) cwCollided = true;
        if (cornerCollisionFlags[2] && !cornerCollisionFlags[3] && !cornerCollisionFlags[0]) cwCollided = true;
        if (cornerCollisionFlags[3] && !cornerCollisionFlags[2] && !cornerCollisionFlags[1]) ccwCollided = true;

        buildEdgePoints();

        for (int i = 0; i < TOTAL_EDGE_POINTS; i++) {
            double x = edgeBuffer[i].getX();
            double y = edgeBuffer[i].getY();

            if (checkCollision(y, Dimensions.getCollisionValuesForY(x))) {
                if (isCloserToLow(y, Dimensions.getCollisionValuesForY(x))) yPlusCollided = true;
                else yMinusCollided = true;
            }
            if (checkCollision(x, Dimensions.getCollisionValuesForX(y))) {
                if (isCloserToLow(x, Dimensions.getCollisionValuesForX(y))) xPlusCollided = true;
                else xMinusCollided = true;
            }
        }

        cornerPosePublisher.set(cornerBuffer);
        edgePosePublisher.set(edgeBuffer);

        return new boolean[] { xMinusCollided, xPlusCollided, yMinusCollided, yPlusCollided, ccwCollided, cwCollided };
    }

    private void buildEdgePoints() {
        final Pose2d[] ordered = { cornerBuffer[0], cornerBuffer[1], cornerBuffer[3], cornerBuffer[2] };

        for (int i = 0; i < CORNER_COUNT; i++) {
            Pose2d a = ordered[i];
            Pose2d b = ordered[(i + 1) % CORNER_COUNT];
            double ax = a.getX(), ay = a.getY();
            double dx = b.getX() - ax, dy = b.getY() - ay;
            Rotation2d rot = a.getRotation();
            int base = i * EDGE_POINTS_PER_SIDE;

            for (int j = 0; j < EDGE_POINTS_PER_SIDE; j++) {
                double t = (j + 1) / (EDGE_POINTS_PER_SIDE + 1.0);
                edgeBuffer[base + j] = new Pose2d(ax + t * dx, ay + t * dy, rot);
            }
        }
    }

    private static boolean checkCollision(double val, double[][] ranges) {
        for (double[] range : ranges) {
            if (range[0] == -1.0 && range[1] == -1.0) continue;
            if (val > range[0] && val < range[1]) return true;
        }
        return false;
    }

    private static boolean isCloserToLow(double val, double[][] ranges) {
        for (double[] range : ranges) {
            if (range[0] == -1.0 && range[1] == -1.0) continue;
            if (val > range[0] && val < range[1]) {
                return (val - range[0]) < (range[1] - val);
            }
        }
        return false;
    }

    public void reset() {
        if (m_swerveDrivetrain == null) return;
        Pose2d startPose = m_swerveDrivetrain.getInitialStartPose();
        currentSimPose = startPose;
        prevSimPose = startPose;
        m_swerveDrivetrain.resetPose(currentSimPose);
    }
}
