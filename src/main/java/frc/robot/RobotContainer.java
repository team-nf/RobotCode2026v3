// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimAndPassCommand;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.GoFromLeftTrenchCommand;
import frc.robot.commands.GoFromRightTrenchCommand;
import frc.robot.commands.IdleDeployedCommand;
import frc.robot.commands.IdleRetractedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReturnFromLeftTrenchCommand;
import frc.robot.commands.ReturnFromRightTrenchCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.constants.Dimensions;
import frc.robot.constants.States.TheMachineStates.TheMachineState;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Container;
import frc.robot.utils.FuelSim;
import frc.robot.utils.HopperSim;
import frc.robot.utils.ShooterSim;
import frc.robot.utils.SwerveFieldContactSim;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController m_driverController;

  private final CommandSwerveDrivetrain m_drivetrainSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;
  private final FeederSubsystem m_feederSubsystem;
  private final HopperSubsystem m_hopperSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final TheMachine m_theMachine;

  private final SwerveTeleopCommand m_swerveTeleopCommand;

  private final AimAndPassCommand m_aimAndPassCommand;
  private final AimAndShootCommand m_aimAndShootCommand;
  private final IdleRetractedCommand m_idleRetractedCommand;
  private final IdleDeployedCommand m_idleDeployedCommand;
  private final IntakeCommand m_intakeCommand;

  public RobotContainer() {
    Container.isBlue = DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Blue).orElse(true);

    m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    m_drivetrainSubsystem = TunerConstants.createDrivetrain();

    m_shooterSubsystem = new ShooterSubsystem();
    m_feederSubsystem = new FeederSubsystem();
    m_hopperSubsystem = new HopperSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_theMachine = new TheMachine(m_shooterSubsystem, m_hopperSubsystem, m_intakeSubsystem, m_feederSubsystem);

    m_swerveTeleopCommand = new SwerveTeleopCommand(m_drivetrainSubsystem, m_driverController);

    m_aimAndPassCommand = new AimAndPassCommand(m_drivetrainSubsystem, m_driverController, m_theMachine);
    m_aimAndShootCommand = new AimAndShootCommand(m_drivetrainSubsystem, m_driverController, m_theMachine);
    m_idleRetractedCommand = new IdleRetractedCommand(m_theMachine);
    m_idleDeployedCommand = new IdleDeployedCommand(m_theMachine);
    m_intakeCommand = new IntakeCommand(m_theMachine);

    configureBindings();

    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Conf/Auto Chooser", autoChooser);

    if (Robot.isSimulation()) {
      configureSims();
    }
  }

  private void configureBindings() {

    m_drivetrainSubsystem.setDefaultCommand(m_swerveTeleopCommand);

    m_driverController.b().onTrue(m_idleRetractedCommand);
    m_driverController.x().onTrue(m_intakeCommand);

    m_driverController.rightTrigger().whileTrue(
      new ConditionalCommand(
        new AimAndShootCommand(m_drivetrainSubsystem, m_driverController, m_theMachine),
        new AimAndPassCommand(m_drivetrainSubsystem, m_driverController, m_theMachine),
        m_drivetrainSubsystem::isRobotOnTheShootingZone
      )
    ).onFalse(m_idleDeployedCommand);

    m_driverController.leftTrigger().whileTrue(
      new ConditionalCommand(
        new ConditionalCommand(new GoFromLeftTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               new GoFromRightTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               m_drivetrainSubsystem::isRobotOnLeftSide),

        new ConditionalCommand(new ReturnFromLeftTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               new ReturnFromRightTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               m_drivetrainSubsystem::isRobotOnLeftSide),

        m_drivetrainSubsystem::isRobotOnTheShootingZone
      ));

    NamedCommands.registerCommand("IdleRetractedCommand", new IdleRetractedCommand(m_theMachine));
    NamedCommands.registerCommand("IdleDeployedCommand", new IdleDeployedCommand(m_theMachine));
    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(m_theMachine));
    NamedCommands.registerCommand("AimAndPassCommand", new AimAndPassCommand(m_drivetrainSubsystem, m_driverController, m_theMachine));
    NamedCommands.registerCommand("AimAndShootCommand", new AimAndShootCommand(m_drivetrainSubsystem, m_driverController, m_theMachine));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void containerPeriodic() {
    if(Robot.isSimulation()) 
      {
        m_theMachine.calculateSubsytemPoses();
        m_theMachine.publishTelemetry();
      }
    //SmartDashboard.putData(CommandScheduler.getInstance());
    //m_theMachine.publishTelemetry();
  }

  private void configureSims() {
    FuelSim fuelSim = FuelSim.getInstance();
    HopperSim hopperSim = HopperSim.getInstance();
    ShooterSim shooterSim = ShooterSim.getInstance();
    
    SwerveFieldContactSim.getInstance().setSwerveDrivetrain(m_drivetrainSubsystem);
    SwerveFieldContactSim.getInstance().setIntakeDeployedSupplier(() -> m_intakeSubsystem.isIntakeDeployed());

    fuelSim.spawnStartingFuel();

    fuelSim.registerRobot(
            Dimensions.BUMPER_WIDTH.in(Meters),
            Dimensions.BUMPER_LENGTH.in(Meters),
            Dimensions.BUMPER_HEIGHT.in(Meters),
            m_drivetrainSubsystem::getPose,
            m_drivetrainSubsystem::getSpeeds);
    
    fuelSim.registerIntake(
            Dimensions.BUMPER_LENGTH.div(2).in(Meters),
            Dimensions.BUMPER_LENGTH.div(2).plus(Dimensions.HOPPER_EXTENSION_LENGTH).in(Meters),
            -Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            Dimensions.BUMPER_WIDTH.div(2).in(Meters),
            () -> m_theMachine.isAbleToIntake()
                        && hopperSim.isHopperAbleToIntake(),
            hopperSim::addFuelToHopper);

    fuelSim.start();

    hopperSim.setRobotPoseSupplier(m_drivetrainSubsystem::getPose);
    hopperSim.setShouldRemoveFuelSupplier(() -> m_theMachine.isState(TheMachineState.REVERSE));

    shooterSim.setShooterRPSSupplier(m_shooterSubsystem::getFlywheel1Velocity);
    shooterSim.setHoodAngleSupplier(m_shooterSubsystem::getHoodPosition);
    shooterSim.setTurretAngleSupplier(m_shooterSubsystem::getTurretAngleDegrees);
    shooterSim.setRobotPoseSupplier(m_drivetrainSubsystem::getPose);
    shooterSim.setChassisSpeedsSupplier(m_drivetrainSubsystem::getFieldSpeeds);
    shooterSim.setShouldShootSupplier(() -> m_theMachine.isState(TheMachineState.SHOOT) || m_theMachine.isState(TheMachineState.PASS));
  }
}
