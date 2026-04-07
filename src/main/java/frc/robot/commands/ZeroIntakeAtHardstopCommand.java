package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/** Runs intake hardstop zeroing until complete. */
public class ZeroIntakeAtHardstopCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public ZeroIntakeAtHardstopCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.startIntakeHardstopZeroing();
  }

  @Override
  public void execute() {
    intakeSubsystem.updateIntakeHardstopZeroing();
  }

  @Override
  public boolean isFinished() {
    return intakeSubsystem.isIntakeHardstopZeroingComplete();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntakeHardstopZeroing();
  }
}
