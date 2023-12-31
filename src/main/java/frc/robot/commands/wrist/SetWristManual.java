// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.BASE.CONTROL_MODE;
import frc.robot.subsystems.Wrist;

public class SetWristManual extends Command {
  /** Creates a new SetWristManual. */
  Wrist m_wrist;

  double m_output;

  public SetWristManual(Wrist wrist, double output) {
    m_wrist = wrist;
    m_output = output;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setClosedLoopControlMode(CONTROL_MODE.OPEN_LOOP);
    m_wrist.setUserSetpoint(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //    m_wrist.set
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
