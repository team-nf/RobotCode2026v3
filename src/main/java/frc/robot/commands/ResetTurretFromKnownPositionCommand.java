package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot command: reset turret motor position from known absolute encoder position. */
public class ResetTurretFromKnownPositionCommand extends Command {

  public ResetTurretFromKnownPositionCommand(ShooterSubsystem shooterSubsystem) {
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
