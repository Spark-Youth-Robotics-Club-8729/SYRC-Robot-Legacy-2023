// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Robot extends TimedRobot {

  private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final PWMSparkMax m_shooter = new PWMSparkMax(2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
  private final Joystick m_stick = new Joystick(0);
  private final Timer m_timer = new Timer();
  private int count = 0;

  @Override
  public void robotInit() {
    m_rightDrive.setInverted(true);
  }

  @Override
  public void autonomousInit() {
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void autonomousPeriodic() {
    if (count == 0) {
      if (m_timer.get() < 2.0) {
        m_robotDrive.arcadeDrive(0.5, 0.0);
      } else {
        m_robotDrive.stopMotor();
        count++;
        m_timer.reset();
      }
    } else if (count == 1) {
      if (m_timer.get() < 3.0) {
        m_robotDrive.arcadeDrive(-0.5, 0.0);
        if (m_timer.get() < 2.0) {
          m_shooter.set(0.5); // Turn on the shooter for the first 2 seconds of moving backward
        } else {
          m_shooter.set(0.0); // Turn off the shooter after 2 seconds
        }
      } else {
        m_robotDrive.stopMotor();
        count++;
        m_timer.reset();
      }
    } else if (count == 2) {
      if (m_timer.get() < 5.0) {
        m_robotDrive.arcadeDrive(0.5, 0.0);
      } else {
        m_robotDrive.stopMotor();
        count++;
        m_timer.reset();
      }
    } else if (count == 3) {
      if (m_timer.get() < 1.0) {
        m_robotDrive.arcadeDrive(0.0, 0.5);
      } else {
        m_robotDrive.stopMotor();
        count++;
      }
    }
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    m_shooter.set(m_stick.getY());
    m_robotDrive.arcadeDrive(m_stick.getY(), m_stick.getX());
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }
}