// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TheMachine;

/** Keeps the machine in intake mode while held/scheduled. */
public class IntakeCommand extends Command {

  private TheMachine theMachine;

  public IntakeCommand(TheMachine theMachine) {
    this.theMachine = theMachine;
    addRequirements(theMachine.getSubsystems());

  }

  @Override
  public void initialize() {
    // Enter intake state once; subsystem periodic handles continuous behavior.
    theMachine.intake();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Trigger/button release should cancel this command.
    return false;
  }
}
