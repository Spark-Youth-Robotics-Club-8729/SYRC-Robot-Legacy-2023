// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"

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
  m_encoder1.Reset();
  m_encoder2.Reset();

  m_encoder1.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
  m_encoder2.SetDistancePerPulse((3.14159265358 * 6) / 360.0);

  // Ultrasonic 
  ultrasonic_trigger_pin_one.Set(true);

  //Colour Matcher
  m_colorMatcher.AddColorMatch(kBlueCargo);
  m_colorMatcher.AddColorMatch(kRedCargo);
  m_colorMatcher.AddColorMatch(kBlackLine); 
  m_colorMatcher.AddColorMatch(kBlueTarmac);
}

void Robot::RobotPeriodic() {
  
  frc::Color detectedColor = m_colorSensor.GetColor();  
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

  double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.125 * voltage_scale_factor;
  frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);


}

void Robot::AutonomousInit() 
{

m_encoder1.Reset();
m_encoder2.Reset();
ultrasonic_trigger_pin_one.Set(true);


}

void Robot::AutonomousPeriodic() 
{
  // Ultrasonic
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.125 * voltage_scale_factor;
  if (ultrasonic_sensor_range_one > 10) {
    m_robotDrive.ArcadeDrive(0, -0.5);
  }

  // if (matchedColor == kBlueTarmac) {}  //Example Colour Sensor if statement

  
//   if (m_encoder1.GetDistance() < 10 && arrivedDestination == false) {
//     m_robotDrive.ArcadeDrive(0, 0.5);
//     } else { 
//         arrivedDestination = true;
//         if(m_encoder1.GetDistance() > 0) {
//             m_robotDrive.ArcadeDrive(0, -0.5);
//         }
//         else {
//             m_robotDrive.ArcadeDrive(0, 0);
//         }
//     }

    int halfFromHubToCargo;

    //Half to hub to hub
    while(m_encoder1.GetDistance()<halfFromHubToCargo){
        m_robotDrive.ArcadeDrive(0, 0.5);
    }

    //Code below will to be drop ball into hub, worry about that later
    
    //Code to go back to second ball from hub
    while(m_encoder1.GetDistance()<(halfFromHubToCargo * 2)){
        m_robotDrive.ArcadeDrive(0,-0.5);
    }

    //Pick up ball here, will worry about later

    //Code to go back to hub
    while(m_encoder1.GetDistance()<(halfFromHubToCargo * 2)){
        m_robotDrive.ArcadeDrive(0,0.5);
    }
}

void Robot::TeleopInit() {
  m_encoder1.Reset();
  m_encoder2.Reset();
}

void Robot::TeleopPeriodic() {

  // Drive with arcade style
  float xDrive = m_xbox.GetY() * 0.8;
  float yDrive = m_xbox.GetX() * 0.8;
  m_robotDrive.ArcadeDrive(yDrive, xDrive);

  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder 1 Distance: ", m_encoder1.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder 2 Distance: ", m_encoder2.GetDistance());
  frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);
  
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