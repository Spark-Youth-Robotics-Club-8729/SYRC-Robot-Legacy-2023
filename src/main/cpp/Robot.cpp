// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {

  //Reset Motors
  m_leftLeadMotor.RestoreFactoryDefaults();
  m_rightLeadMotor.RestoreFactoryDefaults();
  m_leftFollowMotor.RestoreFactoryDefaults();
  m_rightFollowMotor.RestoreFactoryDefaults();

  //Allowing back motors to follow front motors
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);

  //Default
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Encoders
  m_encoder.Reset();
  m_encoder.SetDistancePerPulse((3.14159265358 * 6) / 360.0);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  fmt::print("Auto selected: {}\n", m_autoSelected);
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

m_encoder.Reset();
}
void Robot::AutonomousPeriodic() 
{
  
  if (m_encoder.GetDistance() < 10 && arrivedDestination == false) {
    m_robotDrive.ArcadeDrive(0, 0.4);
    } else {
        //m_robotDrive.ArcadeDrive(0, 0);
        arrivedDestination = true;
        if(m_encoder.GetDistance() > 0) {
            m_robotDrive.ArcadeDrive(0, -0.4);
        }
        else {
            m_robotDrive.ArcadeDrive(0, 0);
        }
    }
}

void Robot::TeleopInit() {

m_encoder.Reset();

}

void Robot::TeleopPeriodic() {

  // Drive with arcade style
  m_robotDrive.ArcadeDrive(-m_stick.GetY(), m_stick.GetX());

  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder Distance: ", m_encoder.GetDistance());
  frc::sim::EncoderSim m_EncoderSim{m_encoder};
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
