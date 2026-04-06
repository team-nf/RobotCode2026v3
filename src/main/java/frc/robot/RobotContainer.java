// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAndPassCommand;
import frc.robot.commands.AimAndShootCommand;
import frc.robot.commands.GoFromLeftTrenchCommand;
import frc.robot.commands.GoFromRightTrenchCommand;
import frc.robot.commands.GoToMidCommand;
import frc.robot.commands.IdleDeployedCommand;
import frc.robot.commands.IdleRetractedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReturnFromLeftTrenchCommand;
import frc.robot.commands.ReturnFromRightTrenchCommand;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.commands.TestMachineCommand;
import frc.robot.commands.AutoCommands.AimAndPassAutoCommand;
import frc.robot.commands.AutoCommands.AimAndShootAutoCommand;
import frc.robot.constants.Dimensions;
import frc.robot.constants.TheMachineConstants;
import frc.robot.constants.States.TheMachineStates.TheMachineState;
import frc.robot.constants.TelemetryConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.AllianceUtil;
import frc.robot.utils.FuelSim;
import frc.robot.utils.HopperSim;
import frc.robot.utils.ShooterSim;
import frc.robot.utils.SwerveFieldContactSim;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Central wiring point for subsystems, commands, driver controls, and autonomous options.
 *
 * <p>If you are onboarding, start here: this file shows which button triggers which behavior and
 * how autonomous commands are exposed to the dashboard.
 */
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
  private final TestMachineCommand m_testMachineCommand;

  // Sim telemetry is intentionally throttled to avoid NetworkTables bandwidth/GC spikes.
  private static final double SIM_TELEMETRY_PERIOD_SEC = 0.05; // 20 Hz
  private double nextSimTelemetryTimeSec = 0.0;
  private boolean sysIdCommandsPublished = false;

  public RobotContainer() {
    // Resolve alliance once at startup for mirrored-field calculations.
    AllianceUtil.refreshAllianceFromDriverStation();

    m_driverController =
        new CommandXboxController(TheMachineConstants.DRIVER_CONTROLLER_PORT_ID);

    m_drivetrainSubsystem = TunerConstants.createDrivetrain();

    m_shooterSubsystem = new ShooterSubsystem(m_drivetrainSubsystem::getHeading);
  m_feederSubsystem = new FeederSubsystem(m_shooterSubsystem::getFlywheelGoalVelocityRps);
    m_hopperSubsystem = new HopperSubsystem();
    m_intakeSubsystem = new IntakeSubsystem();
    m_theMachine = new TheMachine(m_shooterSubsystem, m_hopperSubsystem, m_intakeSubsystem, m_feederSubsystem, m_drivetrainSubsystem::getPose);

    m_swerveTeleopCommand = new SwerveTeleopCommand(m_drivetrainSubsystem, m_driverController);

    m_aimAndPassCommand = new AimAndPassCommand(m_drivetrainSubsystem, m_driverController, m_theMachine);
    m_aimAndShootCommand = new AimAndShootCommand(m_drivetrainSubsystem, m_driverController, m_theMachine);
    m_idleRetractedCommand = new IdleRetractedCommand(m_theMachine);
    m_idleDeployedCommand = new IdleDeployedCommand(m_theMachine);
    m_intakeCommand = new IntakeCommand(m_theMachine);
    m_testMachineCommand = new TestMachineCommand(m_theMachine);

    configureBindings();

    // Toggle this to true to expose only autos prefixed with "comp" during events.
    boolean isCompetition = false;
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );

    SmartDashboard.putData("Conf/Auto Chooser", autoChooser);
    // SysId commands are intentionally not published at startup to reduce dashboard load.

    if (Robot.isSimulation()) {
      configureSims();
    }
  }

  /** Publish SysId dashboard commands on demand (typically test mode only). */
  public void publishSysIdCommands() {
    if (sysIdCommandsPublished) {
      return;
    }
    configureSysIdCommands();
    sysIdCommandsPublished = true;
  }

  private void configureBindings() {
    // Default: field-centric teleop drive runs whenever no other drivetrain command is scheduled.
    m_drivetrainSubsystem.setDefaultCommand(m_swerveTeleopCommand);

    // B -> stow intake and settle machine.
    m_driverController.b().onTrue(m_idleRetractedCommand);

    // X -> begin intake state machine.
    m_driverController.x().onTrue(m_intakeCommand);

  // Left trigger:
    // - In shooting zone: aim + shoot
    // - Elsewhere: aim + pass
    // On release: return to deployed idle (safe transition).
    m_driverController.leftTrigger().whileTrue(
      new ConditionalCommand(
        m_aimAndShootCommand,
        m_aimAndPassCommand,
        m_drivetrainSubsystem::isRobotOnTheShootingZone
      )
    ).onFalse(m_idleDeployedCommand);

    // Y -> pathfind to midfield alignment preset. Release returns to stowed idle.
    m_driverController.y().whileTrue(new GoToMidCommand(m_drivetrainSubsystem, m_theMachine)).onFalse(m_idleRetractedCommand);

    // A -> run NetworkTables-driven machine test mode while held.
    m_driverController.a().whileTrue(m_testMachineCommand).onFalse(m_idleRetractedCommand);

  // Left bumper behavior depends on current zone:
    // - From shooting zone, go out to trench pickup path (left/right based on side)
    // - Outside shooting zone, return from trench back toward shooting area
    m_driverController.leftBumper().whileTrue(
      new ConditionalCommand(
        new ConditionalCommand(new GoFromLeftTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               new GoFromRightTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               m_drivetrainSubsystem::isRobotOnLeftSide),

        new ConditionalCommand(new ReturnFromLeftTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               new ReturnFromRightTrenchCommand(m_drivetrainSubsystem, m_theMachine),
                               m_drivetrainSubsystem::isRobotOnLeftSide),

        m_drivetrainSubsystem::isRobotOnTheShootingZone
      ));

    // Named commands are used by PathPlanner autos.
    // Keep names stable once autos are authored to avoid broken references.
    NamedCommands.registerCommand("IdleRetractedNC", new IdleRetractedCommand(m_theMachine));
    NamedCommands.registerCommand("IdleDeployedNC", new IdleDeployedCommand(m_theMachine));
    NamedCommands.registerCommand("IntakeNC", new IntakeCommand(m_theMachine));
    NamedCommands.registerCommand("AimAndPassNC", new AimAndPassAutoCommand(m_drivetrainSubsystem, m_driverController, m_theMachine));
    NamedCommands.registerCommand("AimAndShootNC", new AimAndShootAutoCommand(m_drivetrainSubsystem, m_driverController, m_theMachine));
    NamedCommands.registerCommand("TestMachineNC", new TestMachineCommand(m_theMachine));
    NamedCommands.registerCommand("WaitForHoodToBeClosedNC", new WaitUntilCommand(m_shooterSubsystem::isHoodClosed));
    NamedCommands.registerCommand("AutoShootSequenceNC", 
      new SequentialCommandGroup(
        new AimAndShootAutoCommand(m_drivetrainSubsystem, m_driverController, m_theMachine).withTimeout(5),
        new IdleDeployedCommand(m_theMachine).until(m_shooterSubsystem::isHoodClosed)
      )
    );
  }

  public Command getAutonomousCommand() {
    // The chooser is published on SmartDashboard as "Conf/Auto Chooser".
    return autoChooser.getSelected();
  }

  public void containerPeriodic() {
    // Sim-only pose publishing for AdvantageScope / field visualization.
    if (Robot.isSimulation()) {
      double nowSec = Timer.getFPGATimestamp();
      if (nowSec >= nextSimTelemetryTimeSec) {
        nextSimTelemetryTimeSec = nowSec + SIM_TELEMETRY_PERIOD_SEC;
        m_theMachine.calculateSubsystemPoses();
        publishTelemetry();
      }
    }
    else {
      //publishTelemetry();

    }

    // Always run machine periodic, both real robot and simulation.
    m_theMachine.machinePeriodic();
    //SmartDashboard.putData(CommandScheduler.getInstance());
    //m_theMachine.publishTelemetry();
  }

  public void publishTelemetry()
  {
    m_theMachine.publishTelemetry();

  SmartDashboard.putNumber("DistanceToHub", TelemetryConstants.roundTelemetry(m_drivetrainSubsystem.getDistanceToHub()));

  }

  public Command getTestCommand()
  {
    return m_testMachineCommand;
  }

  private void configureSims() {
    // Shared singleton sim models.
    FuelSim fuelSim = FuelSim.getInstance();
    HopperSim hopperSim = HopperSim.getInstance();
    ShooterSim shooterSim = ShooterSim.getInstance();
    
    SwerveFieldContactSim.getInstance().setSwerveDrivetrain(m_drivetrainSubsystem);
    SwerveFieldContactSim.getInstance().setIntakeDeployedSupplier(() -> m_intakeSubsystem.isIntakeDeployed());

    // Spawn initial game pieces and register robot geometry/intake pickup volume.
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

    // Wire subsystem state into simulation behavior.
    hopperSim.setRobotPoseSupplier(m_drivetrainSubsystem::getPose);
    hopperSim.setShouldRemoveFuelSupplier(() -> m_theMachine.isState(TheMachineState.REVERSE));

    shooterSim.setShooterRPSSupplier(m_shooterSubsystem::getFlywheel1Velocity);
    shooterSim.setHoodAngleSupplier(m_shooterSubsystem::getHoodPosition);
    shooterSim.setTurretAngleSupplier(m_shooterSubsystem::getTurretAngleDegrees);
    shooterSim.setRobotPoseSupplier(m_drivetrainSubsystem::getPose);
    shooterSim.setChassisSpeedsSupplier(m_drivetrainSubsystem::getFieldSpeeds);
    shooterSim.setShouldShootSupplier(() -> m_theMachine.isState(TheMachineState.SHOOT) || m_theMachine.isState(TheMachineState.PASS));
  }

  private void configureSysIdCommands() {
  // Recommended run order per mechanism (hold each command while active):
  // 1) QuasiForward   2) QuasiReverse   3) DynamicForward   4) DynamicReverse
  // Notes:
  // - Run one mechanism at a time with the robot safely supported/clear.
  // - Reverse directly after forward helps return mechanisms toward start position.
  // - Start with lower dynamic voltage if a mechanism is aggressive.
  // Shooter
  SmartDashboard.putData("SysId/Shooter/Flywheel/QuasiForward",
    m_shooterSubsystem.flywheelSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Flywheel/QuasiReverse",
    m_shooterSubsystem.flywheelSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Shooter/Flywheel/DynamicForward",
    m_shooterSubsystem.flywheelSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Flywheel/DynamicReverse",
    m_shooterSubsystem.flywheelSysIdDynamic(SysIdRoutine.Direction.kReverse));

  SmartDashboard.putData("SysId/Shooter/Hood/QuasiForward",
    m_shooterSubsystem.hoodSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Hood/QuasiReverse",
    m_shooterSubsystem.hoodSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Shooter/Hood/DynamicForward",
    m_shooterSubsystem.hoodSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Hood/DynamicReverse",
    m_shooterSubsystem.hoodSysIdDynamic(SysIdRoutine.Direction.kReverse));

  SmartDashboard.putData("SysId/Shooter/Turret/QuasiForward",
    m_shooterSubsystem.turretSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Turret/QuasiReverse",
    m_shooterSubsystem.turretSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Shooter/Turret/DynamicForward",
    m_shooterSubsystem.turretSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Shooter/Turret/DynamicReverse",
    m_shooterSubsystem.turretSysIdDynamic(SysIdRoutine.Direction.kReverse));

  // Intake
  SmartDashboard.putData("SysId/Intake/Roller/QuasiForward",
    m_intakeSubsystem.intakeRollerSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Intake/Roller/QuasiReverse",
    m_intakeSubsystem.intakeRollerSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Intake/Roller/DynamicForward",
    m_intakeSubsystem.intakeRollerSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Intake/Roller/DynamicReverse",
    m_intakeSubsystem.intakeRollerSysIdDynamic(SysIdRoutine.Direction.kReverse));

  SmartDashboard.putData("SysId/Intake/Arm/QuasiForward",
    m_intakeSubsystem.intakeArmSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Intake/Arm/QuasiReverse",
    m_intakeSubsystem.intakeArmSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Intake/Arm/DynamicForward",
    m_intakeSubsystem.intakeArmSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Intake/Arm/DynamicReverse",
    m_intakeSubsystem.intakeArmSysIdDynamic(SysIdRoutine.Direction.kReverse));

  // Hopper
  SmartDashboard.putData("SysId/Hopper/Main/QuasiForward",
    m_hopperSubsystem.hopperMainSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Hopper/Main/QuasiReverse",
    m_hopperSubsystem.hopperMainSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Hopper/Main/DynamicForward",
    m_hopperSubsystem.hopperMainSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Hopper/Main/DynamicReverse",
    m_hopperSubsystem.hopperMainSysIdDynamic(SysIdRoutine.Direction.kReverse));

  SmartDashboard.putData("SysId/Hopper/Side/QuasiForward",
    m_hopperSubsystem.hopperSideSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Hopper/Side/QuasiReverse",
    m_hopperSubsystem.hopperSideSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Hopper/Side/DynamicForward",
    m_hopperSubsystem.hopperSideSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Hopper/Side/DynamicReverse",
    m_hopperSubsystem.hopperSideSysIdDynamic(SysIdRoutine.Direction.kReverse));

  // Feeder
  SmartDashboard.putData("SysId/Feeder/Belt/QuasiForward",
    m_feederSubsystem.feederBeltSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Feeder/Belt/QuasiReverse",
    m_feederSubsystem.feederBeltSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Feeder/Belt/DynamicForward",
    m_feederSubsystem.feederBeltSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Feeder/Belt/DynamicReverse",
    m_feederSubsystem.feederBeltSysIdDynamic(SysIdRoutine.Direction.kReverse));

  SmartDashboard.putData("SysId/Feeder/Feed/QuasiForward",
    m_feederSubsystem.feederFeedSysIdQuasistatic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Feeder/Feed/QuasiReverse",
    m_feederSubsystem.feederFeedSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  SmartDashboard.putData("SysId/Feeder/Feed/DynamicForward",
    m_feederSubsystem.feederFeedSysIdDynamic(SysIdRoutine.Direction.kForward));
  SmartDashboard.putData("SysId/Feeder/Feed/DynamicReverse",
    m_feederSubsystem.feederFeedSysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
}
