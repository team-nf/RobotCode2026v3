// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.TheMachine;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.io.IOException;
import org.json.simple.parser.ParseException;

/** Leaves shooting zone via left trench path while putting the machine in intake mode. */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoFromLeftTrenchCommand extends ParallelCommandGroup {

  /** Creates a new GoToNeutralFromTrenchCommand. */
  public GoFromLeftTrenchCommand(CommandSwerveDrivetrain drivetrain, TheMachine theMachine) {
    Command builtCommand;
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("GoFromLeftTrench");

      // Path files are authored in blue-frame coordinates; flip for red alliance.
      // if(!Container.isBlue) path = path.flipPath();

      builtCommand =
          AutoBuilder.pathfindThenFollowPath(path, DriveConstants.PATH_CONSTRAINTS_FOLLOW_PATH);
    } catch (IOException | ParseException | FileVersionException ex) {
      DriverStation.reportError("Failed to load path 'GoFromLeftTrench'", ex.getStackTrace());
      builtCommand =
          new InstantCommand(() -> {}); // Fallback to a no-op command if path loading fails
    }

    InstantCommand theMachineCommand = new InstantCommand(() -> theMachine.intake());
    addRequirements(theMachine.getSubsystems());

    // Run drive path and mechanism action together.
    addCommands(builtCommand, theMachineCommand);
  }
}
