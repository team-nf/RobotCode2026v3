package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot command: move turret to closest left 45-degree analog step. */
public class TurretStepClosestLeftCommand extends Command {

  public TurretStepClosestLeftCommand(ShooterSubsystem shooterSubsystem) {
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    // Intentionally left blank for rollback/debug.
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
