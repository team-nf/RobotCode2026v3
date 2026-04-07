package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Default driver-controlled swerve command.
 *
 * <p>Reads joystick axes each loop and sends open-loop field-centric velocity requests.
 */
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveTeleopCommand extends Command {
  /** Creates a new SwerveTeleopCommand. */
  private double MaxSpeed =
      0.9 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  private double MaxAngularRate =
      RotationsPerSecond.of(1)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private CommandXboxController driverController;
  private CommandSwerveDrivetrain swerveDrivetrain;

  private SlewRateLimiter joyXSlewLimiter = new SlewRateLimiter(25);
  private SlewRateLimiter joyYSlewLimiter = new SlewRateLimiter(25);

  public SwerveTeleopCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveDrivetrain = drivetrain;
    this.driverController = joystick;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Left stick = translation, right stick X = rotation.
    swerveDrivetrain.setControl(
        drive
            .withVelocityX(
                joyXSlewLimiter.calculate(-driverController.getLeftY())
                    * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(
                joyYSlewLimiter.calculate(-driverController.getLeftX())
                    * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(
                -driverController.getRightX()
                    * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Default commands intentionally run until interrupted/replaced.
    return false;
  }
}
