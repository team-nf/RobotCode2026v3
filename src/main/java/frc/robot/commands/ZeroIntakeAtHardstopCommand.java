package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Manually zeros the intake arm at the hardstop.
 *
 * <p>While held: drives the arm in reverse toward the retracted hardstop.
 * When released: seeds the encoder to retracted-zero at whatever position the arm is now at.
 */
public class ZeroIntakeAtHardstopCommand extends Command {

  private final IntakeSubsystem intakeSubsystem;

  public ZeroIntakeAtHardstopCommand(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void execute() {
    intakeSubsystem.driveArmReverseForZeroing();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopArmMotor();
    intakeSubsystem.homeIntakeAtRetractedPosition();
  }
}
