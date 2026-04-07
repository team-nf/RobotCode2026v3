package frc.robot.utils;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class MatchTracker {

  private static final int UPDATE_EVERY_LOOPS = 10;
  private int loopCounter = 0;

  private boolean isBlueHubActive = false;
  private boolean isRedHubActive = false;
  private String firstPhaseStatus = "";

  private double activePhaseDuration = 0;
  private double matchTime = 0;

  private String gameData = "";
  private boolean redInactiveFirst = false;

  private boolean isPractice = false;

  public void updateMatchTracker() {
    loopCounter++;
    if (loopCounter < UPDATE_EVERY_LOOPS) {
      return;
    }
    loopCounter = 0;

    isPractice = !DriverStation.isFMSAttached() && DriverStation.isEnabled();

    if (DriverStation.isAutonomousEnabled()) {
      isBlueHubActive = true;
      isRedHubActive = true;
      activePhaseDuration = 0;
      firstPhaseStatus = "Auto";
      publishStatus();
      return;
    }

    if (!DriverStation.isTeleopEnabled()) {
      isBlueHubActive = false;
      isRedHubActive = false;
      activePhaseDuration = 0;
      firstPhaseStatus = "NotTeleop";
      publishStatus();
      return;
    }

    // We're teleop enabled, compute.
    matchTime = DriverStation.getMatchTime();

    gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its likely early in
    // teleop.
    if (gameData.isEmpty() && !isPractice) {
      isBlueHubActive = true;
      isRedHubActive = true;
      activePhaseDuration = 0;
      firstPhaseStatus = "Empty";
      publishStatus();
      return;
    }
    redInactiveFirst = false;
    if (!isPractice) {
      switch (gameData.charAt(0)) {
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> {
          // If we have invalid game data, assume hub is active.
          isBlueHubActive = true;
          isRedHubActive = true;
          firstPhaseStatus = "InvalidData";
          publishStatus();
          return;
        }
      }
    } else {
      // In practice, infer first phase from selected alliance (blue => red inactive first).
      redInactiveFirst =
          DriverStation.getAlliance()
              .map(alliance -> alliance == DriverStation.Alliance.Blue)
              .orElse(Boolean.TRUE.equals(Container.isBlue));
    }

    firstPhaseStatus = (redInactiveFirst ? "BlueFirst" : "RedFirst");

    if (isPractice) matchTime = matchTime - 30;

    if (matchTime > 130) {
      // Transition shift, hub is active.
      isBlueHubActive = true;
      isRedHubActive = true;
      activePhaseDuration = matchTime - 130;
      publishStatus();
      return;
    } else if (matchTime > 105) {
      // Shift 1
      isBlueHubActive = redInactiveFirst;
      isRedHubActive = !redInactiveFirst;
      activePhaseDuration = matchTime - 105;
      publishStatus();
      return;
    } else if (matchTime > 80) {
      // Shift 2
      isBlueHubActive = !redInactiveFirst;
      isRedHubActive = redInactiveFirst;
      activePhaseDuration = matchTime - 80;
      publishStatus();
      return;
    } else if (matchTime > 55) {
      // Shift 3
      isBlueHubActive = redInactiveFirst;
      isRedHubActive = !redInactiveFirst;
      activePhaseDuration = matchTime - 55;
      publishStatus();
      return;
    } else if (matchTime > 30) {
      // Shift 4
      isBlueHubActive = !redInactiveFirst;
      isRedHubActive = redInactiveFirst;
      activePhaseDuration = matchTime - 30;
      publishStatus();
      return;
    } else {
      // End game, hub always active.
      isBlueHubActive = true;
      isRedHubActive = true;
      activePhaseDuration = matchTime;
      publishStatus();
      return;
    }
  }

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("MatchTracker");

  private BooleanPublisher isBlueHubActivePublisher =
      table.getBooleanTopic("IsBlueHubActive").publish();
  private BooleanPublisher isRedHubActivePublisher =
      table.getBooleanTopic("IsRedHubActive").publish();
  private DoublePublisher activePhaseDurationPublisher =
      table.getDoubleTopic("ActivePhaseDuration").publish();
  private DoublePublisher matchTimePublisher = table.getDoubleTopic("GameTime").publish();
  private StringPublisher firstPhaseStatusPublisher = table.getStringTopic("Status").publish();

  public void publishStatus() {
    isBlueHubActivePublisher.set(isBlueHubActive);
    isRedHubActivePublisher.set(isRedHubActive);
    activePhaseDurationPublisher.set(activePhaseDuration);
    matchTimePublisher.set(matchTime);
    firstPhaseStatusPublisher.set(firstPhaseStatus);
  }
}
