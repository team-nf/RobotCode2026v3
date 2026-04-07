package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

/** One-shot command: move turret to closest right 45-degree analog step. */
public class TurretStepClosestRightCommand extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private double turretStartPos;

  public TurretStepClosestRightCommand(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    turretStartPos = shooterSubsystem.getTurretMotorPos();
  }

  @Override
  public void execute() {
    shooterSubsystem.moveTurretRaw(
        turretStartPos + ((30.0 / 360.0) * ShooterConstants.TURRET_GEAR_REDUCTION));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
