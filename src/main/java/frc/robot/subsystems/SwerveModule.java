// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.utils.CtreUtils.configureTalonFx;
import static frc.robot.utils.ModuleMap.MODULE_POSITION;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SWERVE.DRIVE;
import frc.robot.constants.SWERVE.MODULE;
import frc.robot.simulation.MotorSim;
import frc.robot.utils.CtreUtils;
import frc.robot.utils.ModuleMap;
import frc.robot.visualizer.SwerveModuleVisualizer;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase implements AutoCloseable {
  private final ModuleMap.MODULE_POSITION m_modulePosition;
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_angleEncoder;

  private final double m_angleOffset;
  private double m_lastAngle;
  private Pose2d m_pose;
  private boolean m_initSuccess = false;
  private SwerveModuleState m_desiredState;

  private final DutyCycleOut driveMotorDutyControl = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0);
  private final PositionVoltage turnPositionControl = new PositionVoltage(0);
  private final StaticBrake brakeControl = new StaticBrake();
  private final NeutralOut neutralControl = new NeutralOut();

  private final StatusSignal<Double> m_drivePosition;
  private final StatusSignal<Double> m_driveVelocity;
  private final StatusSignal<Double> m_turnPosition;
  private final StatusSignal<Double> m_turnVelocity;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          MODULE.ksDriveVoltSecondsPerMeter,
          MODULE.kvDriveVoltSecondsSquaredPerMeter,
          MODULE.kaDriveVoltSecondsSquaredPerMeter);

  private TalonFXSimState m_turnMotorSimState;
  private TalonFXSimState m_driveMotorSimState;
  private CANcoderSimState m_angleEncoderSimState;

  private SwerveModuleVisualizer m_moduleVisualizer;
  //  private DCMotorSim m_turnMotorModel =
  //      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, .001);
  //
  //  private DCMotorSim m_driveMotorModel =
  //      new DCMotorSim(
  //          // Sim Values
  //          MODULE.kDriveGearbox, MODULE.kDriveMotorGearRatio, 0.2);

  private DCMotorSim m_turnMotorSim =
      new DCMotorSim(MODULE.kTurnGearbox, MODULE.kTurnMotorGearRatio, 0.5);
  //  private FlywheelSim m_driveMotorSim = new
  // FlywheelSim(LinearSystemId.identifyVelocitySystem(0.134648227, 0.002802309),
  //          MODULE.kDriveGearbox, MODULE.kDriveMotorGearRatio);
  private MotorSim m_driveMotorSim;

  public SwerveModule(
      MODULE_POSITION modulePosition,
      TalonFX turnMotor,
      TalonFX driveMotor,
      CANcoder angleEncoder,
      double angleOffset) {
    m_modulePosition = modulePosition;
    m_turnMotor = turnMotor;
    m_driveMotor = driveMotor;
    m_angleEncoder = angleEncoder;
    m_angleOffset = RobotBase.isReal() ? angleOffset : 0;

    var turnMotorConfig = CtreUtils.generateTurnMotorConfig();
    turnMotorConfig.Feedback.SensorToMechanismRatio = MODULE.kTurnMotorGearRatio;
    configureTalonFx(m_turnMotor, turnMotorConfig);

    var driveMotorConfig = CtreUtils.generateDriveMotorConfig();
    driveMotorConfig.Feedback.SensorToMechanismRatio = MODULE.kDriveMotorGearRatio;
    configureTalonFx(m_driveMotor, driveMotorConfig);

    m_drivePosition = driveMotor.getPosition().clone();
    m_driveVelocity = driveMotor.getVelocity().clone();
    m_turnPosition = turnMotor.getPosition().clone();
    m_turnVelocity = turnMotor.getVelocity().clone();

    m_lastAngle = getHeadingDegrees();

    m_moduleVisualizer = new SwerveModuleVisualizer(this.getName(), DRIVE.kMaxSpeedMetersPerSecond);

    initSmartDashboard();

    // To distinguish modules in CommandScheduler
    setName("SwerveModule_" + m_modulePosition.ordinal());

    if (!RobotBase.isReal()) {
      m_turnMotorSimState = m_turnMotor.getSimState();
      m_driveMotorSimState = m_driveMotor.getSimState();
      m_angleEncoderSimState = m_angleEncoder.getSimState();

      //      m_turnMotorSim = MotorSim.createSimpleMotor(
      //              m_turnMotor.getDeviceID(),
      //              "Falcon 500",
      //              0,
      //              0.135872794,
      //              0.002802309,
      //              0,
      //              0,
      //              DCMotor.getFalcon500(1)
      //      );
      //
      m_driveMotorSim =
          MotorSim.createSimpleMotor(
              m_driveMotor.getDeviceID(),
              "Falcon 500",
              0,
              0.134648227,
              0.002802309,
              //              2.46330,
              //          0.12872,
              0,
              0,
              MODULE.kDriveGearbox);
    }
    initModuleHeading();

    SmartDashboard.putData(
        "SwerveModule2D_" + m_modulePosition.ordinal(), m_moduleVisualizer.getMechanism2d());
  }

  private void initModuleHeading() {
    System.out.println("Config CANcoder: " + m_modulePosition.ordinal());
    var encoderConfig = CtreUtils.generateCanCoderConfig();
    encoderConfig.MagnetSensor.MagnetOffset = m_angleOffset / 360.0;
    CtreUtils.configureCANCoder(m_angleEncoder, encoderConfig);
    m_angleEncoder.optimizeBusUtilization(255);
    resetAngleToAbsolute();
  }

  public boolean getInitSuccess() {
    return m_initSuccess;
  }

  public MODULE_POSITION getModulePosition() {
    return m_modulePosition;
  }

  public void resetAngleToAbsolute() {
    resetTurnAngle(0);
  }

  public void resetTurnAngle(double angle) {
    double newAngle = getHeadingDegrees() + angle;

    StatusCode turnMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < (RobotBase.isReal() ? 5 : 1); i++) {
      turnMotorStatus = m_turnMotor.setPosition(newAngle / 360.0);
      if (turnMotorStatus.isOK()) break;
      if (RobotBase.isReal()) Timer.delay(0.02);
    }

    if (!turnMotorStatus.isOK()) {
      System.out.println(
          "Could not apply configs to Swerve Turn TalonFX: "
              + m_turnMotor.getDeviceID()
              + ". Error code: "
              + turnMotorStatus);
    }
  }

  public double getHeadingDegrees() {
    m_turnPosition.refresh();
    m_turnVelocity.refresh();
    return 360.0 * BaseStatusSignal.getLatencyCompensatedValue(m_turnPosition, m_turnVelocity)
        - m_angleOffset;
  }

  public Rotation2d getHeadingRotation2d() {
    return Rotation2d.fromDegrees(getHeadingDegrees());
  }

  public double getVelocityMetersPerSecond() {
    m_driveVelocity.refresh();
    return m_driveVelocity.getValue() * MODULE.kWheelDiameterMeters * Math.PI;
  }

  public double getDriveDistanceMeters() {
    m_drivePosition.refresh();
    m_driveVelocity.refresh();
    return BaseStatusSignal.getLatencyCompensatedValue(m_drivePosition, m_driveVelocity)
        * MODULE.kWheelDiameterMeters
        * Math.PI;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    m_desiredState = CtreUtils.optimize(desiredState, getHeadingRotation2d());

    if (isOpenLoop) {
      double percentOutput = m_desiredState.speedMetersPerSecond / DRIVE.kMaxSpeedMetersPerSecond;
      m_driveMotor.setControl(driveMotorDutyControl.withOutput(percentOutput));
    } else {
      double velocityRPS =
          m_desiredState.speedMetersPerSecond / (MODULE.kWheelDiameterMeters * Math.PI);
      m_driveMotor.setControl(
          driveVelocityControl
              .withVelocity(velocityRPS)
              .withFeedForward(feedforward.calculate(velocityRPS)));
    }

    double angle =
        (Math.abs(m_desiredState.speedMetersPerSecond) <= (DRIVE.kMaxSpeedMetersPerSecond * 0.01))
            ? m_lastAngle
            : m_desiredState.angle
                .getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents
    // Jittering.
    m_turnMotor.setControl(turnPositionControl.withPosition(angle / 360.0));
    m_lastAngle = angle;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getHeadingRotation2d());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDriveDistanceMeters(), getHeadingRotation2d());
  }

  public void setModulePose(Pose2d pose) {
    m_pose = pose;
  }

  public Pose2d getModulePose() {
    return m_pose;
  }

  public void setDriveBrake() {
    m_driveMotor.setControl(brakeControl);
  }

  public void setDriveNeutral() {
    m_driveMotor.setControl(neutralControl);
  }

  public void setTurnBrake() {
    m_turnMotor.setControl(brakeControl);
  }

  public void setTurnNeutral() {
    m_turnMotor.setControl(neutralControl);
  }

  private void initSmartDashboard() {}

  private void updateSmartDashboard() {}

  public void updateLog() {
    Logger.recordOutput(
        "Swerve/Module[" + m_modulePosition + "] Encoder Heading",
        m_angleEncoder.getAbsolutePosition().getValue() * 360.0);
    Logger.recordOutput("Swerve/Module[" + m_modulePosition + "] Angle Offset", m_angleOffset);
    Logger.recordOutput(
        "Swerve/Module[" + m_modulePosition + "] Motor Heading", getHeadingDegrees());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    updateLog();

    m_moduleVisualizer.update(getState());
  }

  @Override
  public void simulationPeriodic() {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_turnMotorSim.getCurrentDrawAmps(), m_driveMotorSim.getCurrentDrawAmps()));

    m_turnMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    m_driveMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    m_turnMotorSim.setInputVoltage(MathUtil.clamp(m_turnMotorSimState.getMotorVoltage(), -12, 12));
    m_driveMotorSim.setInputVoltage(
        MathUtil.clamp(m_driveMotorSimState.getMotorVoltage(), -12, 12));

    double dt = RobotTime.getTimeDelta();
    m_turnMotorSim.update(dt);
    m_driveMotorSim.run();

    var turnVelocityRps = m_turnMotorSim.getAngularVelocityRPM() / 60.0;
    var driveVelocityRps =
        m_driveMotorSim.getAngularVelocityRPM() * MODULE.kDriveMotorGearRatio / 60.0;

    m_turnMotorSimState.setRawRotorPosition(
        m_turnMotorSim.getAngularPositionRotations() * MODULE.kTurnMotorGearRatio);
    m_turnMotorSimState.setRotorVelocity(turnVelocityRps * MODULE.kTurnMotorGearRatio);
    m_angleEncoderSimState.setRawPosition(m_turnMotorSim.getAngularPositionRotations());
    m_angleEncoderSimState.setVelocity(turnVelocityRps);
    m_driveMotorSimState.setRawRotorPosition(
        m_driveMotorSim.getAngularPositionRot() * MODULE.kDriveMotorGearRatio);
    m_driveMotorSimState.setRotorVelocity(driveVelocityRps);
  }

  @SuppressWarnings("RedundantThrows")
  @Override
  public void close() throws Exception {
    m_driveMotor.close();
    m_turnMotor.close();
    m_angleEncoder.close();
  }
}
