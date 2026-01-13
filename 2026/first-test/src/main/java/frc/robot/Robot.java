// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final Joystick m_leftStick;
  private final Joystick m_rightStick;

  // Left side motor controllers (CAN IDs)
  private final WPI_VictorSPX m_leftMotor1 = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_leftMotor2 = new WPI_VictorSPX(2);
  
  // Right side motor controllers (CAN IDs)
  private final WPI_VictorSPX m_rightMotor1 = new WPI_VictorSPX(3);
  private final WPI_VictorSPX m_rightMotor2 = new WPI_VictorSPX(4);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // Set up follower motors - motor 2 on each side follows motor 1
    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor1);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor1);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
  }
}
