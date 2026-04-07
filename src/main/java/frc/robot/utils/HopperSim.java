package frc.robot.utils;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.constants.Dimensions;
import java.util.function.Supplier;

public class HopperSim {

  private final double hopperCapacityDeployed = 48;
  private final double fuelDiameter = 0.15;

  private double hopperMaxCapacity = hopperCapacityDeployed;

  private static HopperSim instance = null;

  private double currentHopperLoad = 0;

  private Supplier<Pose2d> robotPoseSupplier;

  private int[] fuelPosesDeployedDimension = {4, 4, 3};
  private int[] fuelPosesRetractedDimension = {2, 4, 3};

  private Pose3d[] fuelPosesDeployed =
      new Pose3d
          [fuelPosesDeployedDimension[0]
              * fuelPosesDeployedDimension[1]
              * fuelPosesDeployedDimension[2]];
  private Pose3d[] fuelPosesRetracted =
      new Pose3d
          [fuelPosesRetractedDimension[0]
              * fuelPosesRetractedDimension[1]
              * fuelPosesRetractedDimension[2]];

  private DoublePublisher hopperLoadPublisher =
      NetworkTableInstance.getDefault().getDoubleTopic("Sim/Hopper/Current Load").publish();
  private StructArrayPublisher<Pose3d> fuelPosePublisher =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Sim/Hopper/Fuel Poses", Pose3d.struct)
          .publish();

  private Supplier<Boolean> shouldRemoveFuel = () -> false;

  private HopperSim() {
    int count = 0;
    for (int k = 0; k < fuelPosesDeployedDimension[2]; k++) {
      for (int j = 0; j < fuelPosesDeployedDimension[1]; j++) {
        for (int i = 0; i < fuelPosesDeployedDimension[0]; i++) {
          double x = i * fuelDiameter;
          double y = 0.225 - j * fuelDiameter;
          double z = 0.26 + k * fuelDiameter;
          Pose3d fuelPose = new Pose3d(new Translation3d(x, y, z), new Rotation3d());

          fuelPosesDeployed[count] = fuelPose;
          count++;
        }
      }
    }

    count = 0;
    for (int k = 0; k < fuelPosesRetractedDimension[2]; k++) {
      for (int j = 0; j < fuelPosesRetractedDimension[1]; j++) {
        for (int i = 0; i < fuelPosesRetractedDimension[0]; i++) {
          double x = i * fuelDiameter;
          double y = 0.225 - j * fuelDiameter;
          double z = 0.26 + k * fuelDiameter;
          Pose3d fuelPose = new Pose3d(new Translation3d(x, y, z), new Rotation3d());

          fuelPosesRetracted[count] = fuelPose;
          count++;
        }
      }
    }
  }

  public static HopperSim getInstance() {
    if (instance == null) {
      instance = new HopperSim();
    }
    return instance;
  }

  private boolean isFpsCorrect = true; // Tracks if the hopper can intake fuel

  public boolean isHopperAbleToIntake() {
    return isHopperNotFull() && isFpsCorrect;
  }

  private long currentTime = System.currentTimeMillis();
  private long lastFuelIntakeTime = 0;
  private double fuelPerSecond = 3.0; // Default FPS limit

  public void addFuelToHopper() {
    if (currentHopperLoad < hopperMaxCapacity) {
      lastFuelIntakeTime = System.currentTimeMillis();
      currentHopperLoad += 1;
    }

    rejectExtraFuels();
  }

  public void updateFPSStatus() {
    currentTime = System.currentTimeMillis();
    double timeSinceLastIntake = (currentTime - lastFuelIntakeTime) / 1000.0;

    double currentFPS =
        timeSinceLastIntake > 0 ? 1.0 / timeSinceLastIntake : Double.POSITIVE_INFINITY;

    if (fuelPerSecond >= currentFPS) {
      isFpsCorrect = true;
    } else {
      isFpsCorrect = false;
    }
  }

  public void removeFuelFromHopper() {
    if (currentHopperLoad > 0) {
      currentHopperLoad -= 1;
    }
  }

  private double fuelPerInstance = 3.0; // Default limit for adding fuel
  private double prevHopperLoad = 0.0;

  private void rejectExtraFuels() {

    double loadDifference = currentHopperLoad - prevHopperLoad;

    if (loadDifference > fuelPerInstance) {

      for (int i = 0; i < (int) ((loadDifference) - fuelPerInstance); i++) {
        removeFuelFromHopper();

        Transform3d robotPoseTransform =
            new Transform3d(
                new Translation3d(
                    robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), 0),
                new Rotation3d(robotPoseSupplier.get().getRotation()));

        double yOffset =
            Dimensions.BUMPER_WIDTH
                .times(Math.random() * 0.8 - 0.4)
                .in(Meters); // Random offset between -1 and 1

        Pose3d intakePose =
            new Pose3d(
                new Translation3d(Dimensions.HOPPER_EXTENSION_LENGTH.in(Meters), yOffset, 0.15),
                new Rotation3d());

        Pose3d worldIntakePose =
            intakePose
                .transformBy(robotPoseTransform)
                .rotateAround(
                    robotPoseTransform.getTranslation(),
                    new Rotation3d(robotPoseSupplier.get().getRotation()));

        Pose3d intakeSpeed = new Pose3d(0.8, 0, 0, new Rotation3d());
        Pose3d worldIntakeSpeed =
            intakeSpeed.rotateBy(new Rotation3d(robotPoseSupplier.get().getRotation()));

        FuelSim.getInstance()
            .spawnFuel(worldIntakePose.getTranslation(), worldIntakeSpeed.getTranslation());
      }

      currentHopperLoad = prevHopperLoad + fuelPerInstance;
    }

    prevHopperLoad = currentHopperLoad;
  }

  public double getCurrentHopperLoad() {
    return currentHopperLoad;
  }

  public boolean isHopperNotFull() {
    return currentHopperLoad < hopperMaxCapacity;
  }

  public void setRobotPoseSupplier(Supplier<Pose2d> poseSupplier) {
    this.robotPoseSupplier = poseSupplier;
  }

  public void setShouldRemoveFuelSupplier(Supplier<Boolean> shouldRemoveFuelSupplier) {
    this.shouldRemoveFuel = shouldRemoveFuelSupplier;
  }

  public void publishHopperFuelPoses() {
    Transform3d robotPoseTransform =
        new Transform3d(
            new Translation3d(robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), 0),
            new Rotation3d(robotPoseSupplier.get().getRotation()));

    Pose3d[] fuelPoses = new Pose3d[(int) currentHopperLoad];
    for (int i = 0; i < fuelPoses.length; i++) {
      Pose3d localFuelPose;
      localFuelPose = fuelPosesDeployed[i];

      Pose3d worldFuelPose =
          localFuelPose
              .transformBy(robotPoseTransform)
              .rotateAround(
                  robotPoseTransform.getTranslation(),
                  new Rotation3d(robotPoseSupplier.get().getRotation()));
      fuelPoses[i] = worldFuelPose;
    }

    fuelPosePublisher.set(fuelPoses);
  }

  public void updateSim() {
    // This method can be expanded to include more complex simulation logic if needed
    updateFPSStatus();
    hopperLoadPublisher.set(currentHopperLoad);

    if (shouldRemoveFuel.get() && currentHopperLoad > 0 && robotPoseSupplier != null) {
      removeFuelFromHopper();

      Transform3d robotPoseTransform =
          new Transform3d(
              new Translation3d(robotPoseSupplier.get().getX(), robotPoseSupplier.get().getY(), 0),
              new Rotation3d(robotPoseSupplier.get().getRotation()));

      double yOffset =
          Dimensions.BUMPER_WIDTH
              .times(Math.random() * 0.8 - 0.4)
              .in(Meters); // Random offset between -1 and 1

      Pose3d intakePose =
          new Pose3d(
              new Translation3d(Dimensions.HOPPER_EXTENSION_LENGTH.in(Meters), yOffset, 0.15),
              new Rotation3d());

      Pose3d worldIntakePose =
          intakePose
              .transformBy(robotPoseTransform)
              .rotateAround(
                  robotPoseTransform.getTranslation(),
                  new Rotation3d(robotPoseSupplier.get().getRotation()));

      Pose3d intakeSpeed = new Pose3d(0.8, 0, 0, new Rotation3d());
      Pose3d worldIntakeSpeed =
          intakeSpeed.rotateBy(new Rotation3d(robotPoseSupplier.get().getRotation()));

      FuelSim.getInstance()
          .spawnFuel(worldIntakePose.getTranslation(), worldIntakeSpeed.getTranslation());
    }

    publishHopperFuelPoses();
  }

  public void reset() {
    currentHopperLoad = 0;
    prevHopperLoad = 0;
    isFpsCorrect = true;
  }
}
