package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/** Sets the drivetrain to neutral (coast/brake) */
public class SetSwerveNeutralMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive m_swerveDrive;

  /**
   * Sets the drivetrain neutral mode (coast/brake).
   *
   * @param swerveDrive The driveTrain used by this command.
   */
  public SetSwerveNeutralMode(SwerveDrive swerveDrive) {
    m_swerveDrive = swerveDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveDrive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveDrive.setNeutral();
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
    return true;
  }
}
