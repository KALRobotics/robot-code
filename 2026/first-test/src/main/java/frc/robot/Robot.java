// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * Basic robot with Spark MAX + NEO motors, tank drive, NavX2, and Logitech F310.
 * - Teleop: Manual tank drive (no gyro/encoders)
 * - Test: Button A = hold rotation with PID + gyro; Button B = hold position 0.5m ahead with PID + encoders
 */
public class Robot extends TimedRobot {

  // Controller - Logitech F310 (A=1, B=2, Left stick Y=axis 1, Right stick Y=axis 5)
  private final Joystick m_driverController = new Joystick(0);

  // Spark MAX + NEO motors - tank drive (2 per side)
  // CAN IDs: adjust for your robot
  private final SparkMax m_leftLeader = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax m_leftFollower = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax m_rightLeader = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax m_rightFollower = new SparkMax(4, MotorType.kBrushless);

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  // NavX2 via SPI (RoboRIO 2.0 MXP port)
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Encoders from NEO built-in
  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  // Tune these for your drivetrain (wheel diameter, gear ratio)
  private static final double ENCODER_REVS_PER_METER = 20.0;

  // Test mode: rotation hold (Button A)
  private double m_targetRotationDeg = 0;
  private boolean m_rotationHoldActive = false;
  private final PIDController m_rotationPid =
      new PIDController(0.02, 0.0, 0.001); // Tune later

  // Test mode: position hold (Button B) - 0.5m ahead
  private double m_targetPositionMeters = 0;
  private boolean m_positionHoldActive = false;
  private final PIDController m_positionPid =
      new PIDController(0.5, 0.0, 0.0); // Tune later

  // Limit PID output when holding against external force - prevents motor stall/burn
  private static final double MAX_HOLD_OUTPUT = 0.5;

  // Teleop: limit max output to reduce current draw and prevent brownouts
  private static final double TELEOP_MAX_OUTPUT = 0.85;

  public Robot() {
    m_rightGroup.setInverted(true);

    m_rotationPid.setTolerance(2.0);
    m_positionPid.setTolerance(0.02);
  }

  @Override
  public void robotInit() {
    // Brownout prevention: lower current limits, ramp rate, voltage compensation
    var motorConfig =
        new SparkMaxConfig()
            .smartCurrentLimit(30, 45) // Lower than NEO max - reduces brownout risk
            .idleMode(SparkBaseConfig.IdleMode.kBrake)
            .openLoopRampRate(0.25) // 0.25s to reach target - smooths current spikes
            .voltageCompensation(12.0); // Compensates as battery voltage drops (12V nominal)
    for (var motor : new SparkMax[] {m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower}) {
      motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
    // NavX2 auto-calibrates on power-up; hold robot still during init
  }

  @Override
  public void teleopPeriodic() {
    double leftY = MathUtil.clamp(-m_driverController.getRawAxis(1), -TELEOP_MAX_OUTPUT, TELEOP_MAX_OUTPUT);
    double rightY = MathUtil.clamp(-m_driverController.getRawAxis(5), -TELEOP_MAX_OUTPUT, TELEOP_MAX_OUTPUT);

    m_drive.tankDrive(leftY, rightY);
  }

  @Override
  public void testInit() {
    m_rotationHoldActive = false;
    m_positionHoldActive = false;
  }

  @Override
  public void testPeriodic() {
    // Button A: set target rotation to current, hold with PID + gyro
    if (m_driverController.getRawButtonPressed(1)) {
      m_targetRotationDeg = m_gyro.getAngle();
      m_rotationHoldActive = true;
      m_positionHoldActive = false;
    }

    // Button B: set target position 0.5m ahead, hold with PID + encoders
    if (m_driverController.getRawButtonPressed(2)) {
      double avgRevs = (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
      m_targetPositionMeters = (avgRevs / ENCODER_REVS_PER_METER) + 0.5;
      m_positionHoldActive = true;
      m_rotationHoldActive = false;
    }

    if (m_rotationHoldActive) {
      double output =
          MathUtil.clamp(
              m_rotationPid.calculate(m_gyro.getAngle(), m_targetRotationDeg),
              -MAX_HOLD_OUTPUT,
              MAX_HOLD_OUTPUT);
      m_drive.arcadeDrive(0, output);
    } else if (m_positionHoldActive) {
      double avgRevs = (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
      double currentMeters = avgRevs / ENCODER_REVS_PER_METER;
      double output =
          MathUtil.clamp(
              m_positionPid.calculate(currentMeters, m_targetPositionMeters),
              -MAX_HOLD_OUTPUT,
              MAX_HOLD_OUTPUT);
      m_drive.arcadeDrive(output, 0);
    } else {
      m_drive.stopMotor();
    }
  }
}
