// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;

/** Holds the machine in a safe stowed-idle state (intake retracted). */
public class IdleRetractedCommand extends Command {

  private TheMachine theMachine;

  public IdleRetractedCommand(TheMachine theMachine) {
    this.theMachine = theMachine;
    addRequirements(theMachine.getSubsystems());

  }

  @Override
  public void initialize() {
    // Enter retracted idle immediately on schedule.
    theMachine.idleRetracted();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Runs continuously until another command takes control.
    return false;
  }
}
