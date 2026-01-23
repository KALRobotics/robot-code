// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.units.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class,
 * specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private float targetAngle;
  private AHRS navx;
  private final DifferentialDrive m_robotDrive;

  private final Joystick m_driverController;

  private final PIDController m_pid = new PIDController(0.02, 0, 0.002);
  // Left side motor controllers (CAN IDs)
  private final WPI_VictorSPX m_leftBackMotor = new WPI_VictorSPX(0);
  private final WPI_VictorSPX m_rightBackMotor = new WPI_VictorSPX(1);

  private final WPI_VictorSPX m_leftFrontMotor = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_rightFrontMotor = new WPI_VictorSPX(3);

  // Right side motor controllers (CAN IDs)

  /** Called once at the beginning of the robot program. */
  public Robot() {

    try {
      // Use SPI.Port.kMXP for a navX2-MXP plugged into the center port
      navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    } catch (RuntimeException ex) {
      System.out.println("Error instantiating navX: " + ex.getMessage());
    }

    // Set up follower motors - motor 2 on each side follows motor 1
    m_leftBackMotor.follow(m_leftFrontMotor);
    m_rightBackMotor.follow(m_rightFrontMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightFrontMotor.setInverted(true);
    m_rightBackMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftFrontMotor, m_rightFrontMotor);
    m_driverController = new Joystick(0);

    SendableRegistry.addChild(m_robotDrive, m_leftFrontMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightFrontMotor);
  }


  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(
        -m_driverController.getRawAxis(1),
        -m_driverController.getRawAxis(5));
  }

  @Override
  public void testInit() {
    // Test modu ilk başladığında yapılacaklar (isteğe bağlı)
  }

  @Override
  public void testPeriodic() {
    // A,B,X,Y
    if (m_driverController.getRawButtonPressed(1)) 
    {
      targetAngle = navx.getYaw() + 10f;
    }
    if (m_driverController.getRawButtonPressed(2)) 
    {
      targetAngle = navx.getYaw();
    }

      m_pid.enableContinuousInput(-180, 180);

      double output = m_pid.calculate(navx.getYaw(), targetAngle);
      m_robotDrive.arcadeDrive(0, output);
  }
}
