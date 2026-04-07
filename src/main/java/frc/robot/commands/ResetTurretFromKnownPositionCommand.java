package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot command: reset turret motor position from known absolute encoder position. */
public class ResetTurretFromKnownPositionCommand extends InstantCommand {

  private final ShooterSubsystem s;

  public ResetTurretFromKnownPositionCommand(ShooterSubsystem shooterSubsystem) {
    s = shooterSubsystem;
  }

  @Override
  public void initialize() {
    s.syncTurretMotorToAbsoluteEncoder();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
