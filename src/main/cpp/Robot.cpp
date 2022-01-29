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

  //Camera
  frc::CameraServer::StartAutomaticCapture();
  
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


  double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  double  ultrasonic_sensor_range_two = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);
  frc::SmartDashboard::PutNumber("Sensor 2 Range", ultrasonic_sensor_range_two);

}

void Robot::AutonomousInit() 
{

m_encoder1.Reset();
m_encoder2.Reset();
ultrasonic_trigger_pin_one.Set(true);
// inRange = true;
// once = true;
// resetOnce=false;
// encodersReset=false;
// outtakeBallSecondTime=false;
pastBlue = 0.0;
currentBlue = 0.0;
distance = true;
cargo_Outtake_Time = 0;
phase=0;
phase4=0;
encoderAverage=0.0;
reset=false;
closetoCargo = false;

}

void Robot::AutonomousPeriodic() 
{

frc::Color detectedColor = m_colorSensor.GetColor();  
frc::SmartDashboard::PutNumber("Red", detectedColor.red);
frc::SmartDashboard::PutNumber("Green", detectedColor.green);
frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
pastBlue = currentBlue;
currentBlue = detectedColor.blue;
int distanceFromTarmac = 0; 

encoderAverage = (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2;
// std::string colorString;
// double confidence = 0.0;
// frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence); //Detecting the colour our sensor sees

  // Route 3
  // if (ultrasonic_sensor_range_one>5) { 
  //   m_robotDrive.ArcadeDrive(0.75, 0);
  // } else {
  //   m_robotDrive.ArcadeDrive(0, 0);
  //   // add motor to shoot
  // }
  // while (matchedColor == kBlueTarmac) {
  //   m_robotDrive.ArcadeDrive(0.8, 0);
  // }

//ROUTE 1:
// Phase 0: Drive from starting position to hub
if (phase == 0) { 
  if (ultrasonic_sensor_range_one > 12 && distance == true) { // Checking if ultrasonic is in range of minimum 12 inches away from hub
    m_robotDrive.ArcadeDrive(0, -0.75); // Drive towards hub
  } 
  else { // Ultrasonic is too close to be precise (12 or less away from hub)
    distance = false;
    if (encoderAverage > -6) { // Use encoders to measure distance now
      m_robotDrive.ArcadeDrive(0, -0.75); // Drive until 6 inches from hub
    } 
    else { // Arrived at desired distance from hub for outtake
      m_robotDrive.ArcadeDrive(0, 0);
      phase = 1;
    }
  }
}
// Phase 1: Outtake ball
if (phase == 1) {
  if (cargo_Outtake_Time < 50) { // 50 multiplied by 20 millisecond period is approximately 1 second
    // Outtake on
    cargo_Outtake_Time++; 
  }
  else { // Done dropping cargo into hub
    // Outtake off
    phase = 2;
    cargo_Outtake_Time = 0;
    cargo_Intake_Time = 0;
  }
}
// Phase 1.5: Drive until outside the tarmac
if (phase == 2 && reset == true) {
  if (ultrasonic_sensor_range_one > 6) {  
    m_robotDrive.ArcadeDrive(0, 0.75);
  }
}
// Phase 2: Drive to edge of tarmac
if (phase == 2 && reset == false) {
  if (currentBlue < pastBlue + 150) {  // If current blue value is within 150 (this range can be tested) of past blue value, we are assuming that the surface being sensed has not changed
    m_robotDrive.ArcadeDrive(0, 0.75); // Continue driving until tarmac outline reached; below we would align robot if we are doing that (maybe)
  } 
  else { // Tarmac outline reached (blue tape sensed) 
    phase = 3;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Prepare for phase 3
  }
} 
// Phase 3: Drive to cargo
if (phase == 3) {
  if (encoderAverage < 350/8 && closetoCargo==false) { // Real distance = 323/8 inches
    m_robotDrive.ArcadeDrive(0, 0.75);
    if (encoderAverage < 270/8) {
      closetoCargo=true;
    }
  }
  if (cargo_Intake_Time < 150) { 
    // Intake on
    cargo_Intake_Time++; 
    m_robotDrive.ArcadeDrive(0, 0.75);
  }
  else { // Done picking up cargo
    // Intake off
    phase = 0;
    distance = true;
    reset = true;
    cargo_Intake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Back to phase 0
  }
}



 // Route 3
 int phase3=0;
 //Phase 0: Drop off ball at terminal
 if (phase3 == 0) { 
  if (cargo_Outtake_Time < 50){
    cargo_Outtake_Time++;
  }
  else {
    phase3 = 2;
    cargo_Outtake_Time = 0;
    cargo_Intake_Time = 0;
  }
 }

 //Phase two: Back up untill you are 34.2 inches away from wall. 
 if(ultrasonic_sensor_range_one>34.2){
   m_robotDrive.ArcadeDrive(0, 0.75);
 }
 //Phase three: use nav sensor to turn 90 degrees
 

// Route 4

// Phase 0: Drive from starting position to hub
int phase4 = 0;
if (phase4 == 0) { 
  if (ultrasonic_sensor_range_one > 12 && distance == true) { // Checking if ultrasonic is in range of minimum 12 inches away from hub
    m_robotDrive.ArcadeDrive(0, -0.75); // Drive towards hub
  } 
  else { // Ultrasonic is too close to be precise (12 or less away from hub)
    distance = false;
    if (encoderAverage > -6) { // Use encoders to measure distance now
      m_robotDrive.ArcadeDrive(0, -0.75); // Drive until 6 inches from hub
    } 
    else { // Arrived at desired distance from hub for outtake
      m_robotDrive.ArcadeDrive(0, 0);
      phase4 = 1;
    }
  }
}
// Phase 1: Outtake ball
if (phase4 == 1) {
  if (cargo_Outtake_Time < 50) { // 50 multiplied by 20 millisecond period is approximately 1 second
    // Outtake on
    cargo_Outtake_Time++; 
  }
  else { // Done dropping cargo into hub
    // Outtake off
    phase4 = 2;
    cargo_Outtake_Time = 0;
    cargo_Intake_Time = 0;
  }
}
// Phase 2: Back up to Black Line
if (phase4 == 2 && reset == false) {
  if (currentBlue < pastBlue + 150) {  // If current blue value is within 150 (this range can be tested) of past blue value, we are assuming that the surface being sensed has not changed
    m_robotDrive.ArcadeDrive(0, 0.75); // Continue driving until tarmac outline reached; below we would align robot if we are doing that (maybe)
  } 
  else { // Tarmac outline reached (blue tape sensed) 
    phase4 = 3;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Prepare for phase 3
  }
} 
// Phase 3
if (phase4 == 3){
  if (encoderAverage<2) {
    m_robotDrive.ArcadeDrive(0.75, 0);
  } else {
    phase4 = 4;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Prepare for phase 4
  }
}
// Phase 4
if (phase == 4) {
  if (encoderAverage < 250/8 && closetoCargo==false) { // Real distance = 323/8 inches
    m_robotDrive.ArcadeDrive(0, 0.75);
    if (encoderAverage < 170/8) {
      closetoCargo=true;
    }
  }
  if (cargo_Intake_Time < 150) { 
    // Intake on
    cargo_Intake_Time++; 
    m_robotDrive.ArcadeDrive(0, 0.75);
  }
  else { // Done picking up cargo
    // Intake off
    phase = 0;
    distance = true;
    reset = true;
    cargo_Intake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Back to phase 0
  }
}
} 
void Robot::TeleopInit() {
  m_encoder1.Reset();
  m_encoder2.Reset();
  
}

void Robot::TeleopPeriodic() {

  // Drive with arcade style
  float xDrive = m_xbox.GetX() * 0.8;
  float yDrive = m_xbox.GetY() * 0.8;
  m_robotDrive.ArcadeDrive(-yDrive, xDrive);

  // frc::DifferentialDriveOdometry m_odometry{m_gyro.GetRotation2d()};
  // m_odometry.Update(m_gyro.GetRotation2d(),
  //                   units::meter_t(m_encoder1.GetDistance()),  
  //                   units::meter_t(m_encoder2.GetDistance()));
  // m_field.SetRobotPose(m_odometry.GetPose());

  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder 1 Distance: ", m_encoder1.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder 2 Distance: ", m_encoder2.GetDistance());
  frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);
  // frc::SmartDashboard::PutData("Field", &m_field);
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