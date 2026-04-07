// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;

/** Holds the machine in idle with intake deployed (ready to acquire quickly). */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReverseCommand extends Command {

  private TheMachine theMachine;

  /** Creates a new IdleDeployedCommand. */
  public ReverseCommand(TheMachine theMachine) {
    this.theMachine = theMachine;
    addRequirements(theMachine.getSubsystems());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Enter deployed idle immediately.
    theMachine.reverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This state should persist until interrupted.
    return false;
  }
}
