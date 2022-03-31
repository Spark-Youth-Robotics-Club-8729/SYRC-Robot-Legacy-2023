// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Libraries
#include "Robot.h"
#include <stdlib.h>
#include <fmt/core.h>

void Robot::RobotInit() {

  //Reset Shooter and Storage motors
  m_shooter.RestoreFactoryDefaults();
  m_storage.RestoreFactoryDefaults();

  //Camera
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetResolution (320, 240);
  
  //Encoders
  m_encoder1.Reset();
  m_encoder2.Reset();
  m_encoder1.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
  m_encoder2.SetDistancePerPulse((3.14159265358 * 6) / 360.0);

  // Ultrasonic 
  // ultrasonic_trigger_pin_one.Set(true);

  //Default
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);

  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

m_autoSelected = m_chooser.GetSelected();
fmt::print("Auto selected: {}\n", m_autoSelected);

m_encoder1.Reset();
m_encoder2.Reset();

phase=1;
encoderAverage=0.0;

}


void Robot::AutonomousPeriodic() 

{

frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);
frc::SmartDashboard::PutNumber("Encoder 1: ", (m_encoder1.GetDistance()));

//Encoder Periodic
encoderAverage = (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2;

if (phase == 1) {
  if (time < 100) { 
    m_robotDrive.ArcadeDrive(0, 0.55); 
    intake.Set(-0.80);
    time++;
  } 
  else { 
    phase = 2;
    time=0;
  }

}
if (phase==2) {
  if (time < 50) {
  m_shooter.Set(0.67);
  time++;
  }
  else {
    phase=3;
    time=0;
  }
}

if (phase == 3) {
  if (time < 50) {
    m_robotDrive.ArcadeDrive(0, -0.55);
    time++;
  }
  else {
    phase = 4;
    time=0;
  }
}
if (phase == 4) {
    intake.Set(0.0);
    m_robotDrive.ArcadeDrive(0, 0);
    if (time < 150) {
      time++;
    if (time > 50) {
    m_storage.Set(-0.95);
    }
  }
  else {
    phase=5;
    m_shooter.Set(0.0);
    m_storage.Set(0.0);
    time=0;
  }
}
}

void Robot::TeleopInit() {

  m_encoder1.Reset();
  m_encoder2.Reset();

}

void Robot::TeleopPeriodic() {

  Intake();
  Storage();
  Outtake();
  Movement();
  Hanging1();
  SmartDashboard();
  Camera();

}

void Robot::Intake() {

  if (m_xbox.GetRawButton(1)) {
    intake.Set(-0.80);
  }

  if (m_xbox.GetRawButton(3)) {
    intake.Set(0.0);
  }

  if (m_xbox.GetRawButton(9)) {

    intake.Set(0.80);

  }

}

void Robot::Hanging1() { 

OuterLeftClimber.Set(m_xbox.GetRawAxis(3)*0.95);
OuterRightClimber.Set(m_xbox.GetRawAxis(3)*0.95);
if (m_xbox.GetRawButton(11)) {
  InnerClimberLateral.Set(0.40);
  OuterClimberLateral.Set(0.40);
}

if (m_xbox.GetRawButton(12)) {
  InnerClimberLateral.Set(0.0);
  OuterClimberLateral.Set(0.0);
}

}

void Robot::Movement() {

  float xDrive = m_stick.GetRawAxis(4);
  float yDrive = (m_stick.GetRawAxis(1) *-1.0);
  m_robotDrive.ArcadeDrive(xDrive, yDrive);

}

void Robot::SmartDashboard() {

  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder 1 Distance: ", m_encoder1.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder 2 Distance: ", m_encoder2.GetDistance());
  frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);
  frc::SmartDashboard::PutNumber("Shooter RPM", m_ShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Shooter Speed", m_shooter.Get());
  frc::SmartDashboard::PutNumber("Feeder RPM", m_FeederEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Feeder Speed", m_storage.Get());

  //Limelight
  std::shared_ptr table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  targetArea = table->GetNumber("ta",0.0);

}

void Robot::Storage() {

  if (m_xbox.GetRawButton(6)) {
    m_storage.Set(-0.95); 
  } 

  if (m_xbox.GetRawButton(8)) { 
    m_storage.Set(-0.65); 
  }
  
  if (m_xbox.GetRawButton(2)) {
    m_storage.Set(0.0);
  }

}

void Robot::Outtake() {

  if (m_xbox.GetRawButton(5)) {
    m_shooter.Set(0.65); 
  } 

  if (m_xbox.GetRawButton(7)) { 
    m_shooter.Set(0.50); 
  }

  if (m_xbox.GetRawButton(4)) {
    m_shooter.Set(0.0);
  }

}

void Robot::Camera() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif