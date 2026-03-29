// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.TheMachine;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.Container;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToMidCommand extends ParallelCommandGroup {

  /** Creates a new GoToNeutralFromTrenchCommand. */
  public GoToMidCommand(CommandSwerveDrivetrain drivetrain, TheMachine theMachine) {
    Command builtCommand = AutoBuilder.pathfindToPose(new Pose2d(8,4, new Rotation2d(Math.toRadians(90))),
                                                     DriveConstants.PATH_CONSTRAINTS_FOLLOW_PATH);
    InstantCommand theMachineCommand = new InstantCommand(() -> theMachine.idleRetracted());
    addRequirements(theMachine.getSubsystems());
    addCommands(builtCommand, theMachineCommand);
  }

}
