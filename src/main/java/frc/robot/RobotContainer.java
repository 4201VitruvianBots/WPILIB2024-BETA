// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.flywheel.RunFlywheel;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.kicker.RunKickerIn;
import frc.robot.commands.kicker.RunKickerOut;
import frc.robot.commands.swerve.SetSwerveDrive;
import frc.robot.commands.wrist.SetWristSetpoint;
import frc.robot.constants.BASE;
import frc.robot.constants.BASE.SETPOINT;
import frc.robot.constants.INTAKE;
import frc.robot.constants.USB;
import frc.robot.simulation.FieldSim;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive m_swerveDrive = new SwerveDrive();
  private final Wrist m_wrist = new Wrist();
  private final IntakeShooter m_intakeShooter = new IntakeShooter();
  private final FieldSim m_fieldSim = new FieldSim(m_swerveDrive);

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private final Joystick leftJoystick = new Joystick(USB.leftJoystick);
  private final Joystick rightJoystick = new Joystick(USB.rightJoystick);
  private final CommandXboxController xboxController =
      new CommandXboxController(USB.xBoxController);

  private final Trigger[] leftJoystickTriggers = new Trigger[2]; // left joystick buttons
  private final Trigger[] rightJoystickTriggers = new Trigger[2]; // right joystick buttons

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    // Configure the trigger bindings
    configureBindings();

    initializeAutoChooser();
  }

  public void initializeSubsystems() {
    if (RobotBase.isReal()) {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> leftJoystick.getRawAxis(1),
              () -> leftJoystick.getRawAxis(0),
              () -> rightJoystick.getRawAxis(0)));
    } else {
      m_swerveDrive.setDefaultCommand(
          new SetSwerveDrive(
              m_swerveDrive,
              () -> -leftJoystick.getRawAxis(1),
              () -> -leftJoystick.getRawAxis(0),
              () -> -leftJoystick.getRawAxis(2)));
    }
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    for (int i = 0; i < leftJoystickTriggers.length; i++)
      leftJoystickTriggers[i] = new JoystickButton(leftJoystick, (i + 1));
    for (int i = 0; i < rightJoystickTriggers.length; i++)
      rightJoystickTriggers[i] = new JoystickButton(rightJoystick, (i + 1));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    xboxController
        .leftTrigger()
        .whileTrue(
            new RunIntake(m_intakeShooter)
                .alongWith(new SetWristSetpoint(m_wrist, SETPOINT.INTAKING_LOW_CUBE)));

    xboxController.x().whileTrue(new RunKickerOut(m_intakeShooter));

    xboxController.rightTrigger().whileTrue(new RunKickerIn(m_intakeShooter));

    xboxController
        .y()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.HIGH)
                .alongWith(new SetWristSetpoint(m_wrist, SETPOINT.SCORE_HIGH_CUBE)));

    xboxController
        .b()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.MEDIUM)
                .alongWith(new SetWristSetpoint(m_wrist, SETPOINT.SCORE_MID_CUBE)));

    xboxController
        .a()
        .whileTrue(
            new RunFlywheel(m_intakeShooter, INTAKE.FLYWHEEL_SPEED.LOW)
                .alongWith(new SetWristSetpoint(m_wrist, SETPOINT.INTAKING_LOW_CUBE)));

    xboxController.povDown().whileTrue(new SetWristSetpoint(m_wrist, SETPOINT.STOWED));

    xboxController
        .leftBumper()
        .whileTrue((new SetWristSetpoint(m_wrist, BASE.SETPOINT.INTAKING_LOW_CUBE)));
  }

  public void initializeAutoChooser() {
    // m_autoChooser.setDefaultOption(
    //     "SubstationThree", new SubstationThree("SubstationTwoPickup", m_swerveDrive,
    // m_fieldSim));
    // m_autoChooser.addOption("DriveStraight", new DriveStraight(m_swerveDrive, m_fieldSim));
    m_autoChooser.setDefaultOption("Nothing", new WaitCommand(0));
    SmartDashboard.putData("AutoChooser", m_autoChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  public void periodic() {
    // Absolute definition of jank right here. Please change this before Beach Blitz
    // :nate:
    if (Math.abs((MathUtil.applyDeadband(xboxController.getLeftX(), 0.05))) > 0) {
      //      RunCommand(new SetWristManual(m_wrist, xboxController::getLeftX()));
    }
  }

  public void simulationPeriodic() {
    m_fieldSim.periodic();
  }
}
