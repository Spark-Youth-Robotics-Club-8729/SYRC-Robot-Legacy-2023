// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <stdlib.h>
#include <fmt/core.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include "rev/ColorSensorV3.h"

void Robot::RobotInit() {

  //Reset Motors
  m_leftLeadMotor.RestoreFactoryDefaults();
  m_rightLeadMotor.RestoreFactoryDefaults();
  m_leftFollowMotor.RestoreFactoryDefaults();
  m_rightFollowMotor.RestoreFactoryDefaults();
  m_shooter1.RestoreFactoryDefaults();
  m_shooter2.RestoreFactoryDefaults();
  m_storage.RestoreFactoryDefaults();


  //Allowing back motors to follow front motors
  m_leftFollowMotor.Follow(m_leftLeadMotor);
  m_rightFollowMotor.Follow(m_rightLeadMotor);
  m_shooter2.Follow(m_shooter1);


  //Default
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Camera
  cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
  camera.SetResolution (640, 480);
  
  //Encoders
  m_encoder1.Reset();
  m_encoder2.Reset();

  m_encoder1.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
  m_encoder2.SetDistancePerPulse((3.14159265358 * 6) / 360.0);

  // Ultrasonic 
  ultrasonic_trigger_pin_one.Set(true);

  //Colour Matcher
  // m_colorMatcher.AddColorMatch(kBlueCargo);
  // m_colorMatcher.AddColorMatch(kRedCargo);
  // m_colorMatcher.AddColorMatch(kBlackLine); 
  // m_colorMatcher.AddColorMatch(kBlueTarmac);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() 
{
//Reset/Initializing Sensors
m_encoder1.Reset();
m_encoder2.Reset();
ultrasonic_trigger_pin_one.Set(true);

//Variables (bools, phases)

// inRange = true;
// once = true;
// resetOnce=false;
// encodersReset=false;
// outtakeBallSecondTime=false;
currentRed = 0.0;
currentBlue = 0.0;
distance = true;
cargo_Outtake_Time = 0;
phase=0;
phase4=0;
phase3=0;
encoderAverage=0.0;
reset=false;
closetoCargo = false;
increment = 0;
SenseColour = false;
}

void Robot::AutonomousPeriodic() 
{

//Colour Sensor Periodic
frc::Color detectedColor = m_colorSensor.GetColor();  
frc::SmartDashboard::PutNumber("Red", detectedColor.red);
frc::SmartDashboard::PutNumber("Green", detectedColor.green);
frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
currentRed = detectedColor.red;
currentBlue = detectedColor.blue;

//Ultrasonic Sensor Periodic
double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);
frc::SmartDashboard::PutNumber("NAV sensor", (m_gyro.GetAngle()));
//Encoder Periodic
encoderAverage = (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2;

//********************************************************************************************************************************

//ROUTE 1:

// Phase 0: Drive from starting position to hub
if (phase == 0) { 
  if (ultrasonic_sensor_range_one > 20) { // Checking if ultrasonic is in range of minimum 12 inches away from hub
    m_robotDrive.ArcadeDrive(0, 0.50); // Drive towards hub
    m_encoder1.Reset();
    m_encoder2.Reset();
  } 
  else { // Ultrasonic is too close to be precise (12 or less away from hub)
    // distance = false;
    // if (encoderAverage > -6) { // Use encoders to measure distance now
    //   m_robotDrive.ArcadeDrive(0, 0.50); // Drive until 6 inches from hub
    // } 
    phase = 1;
    
  }
}
// Phase 1: Outtake ball
if (phase == 1) {
  if (cargo_Outtake_Time < 50) { // 50 multiplied by 20 millisecond period is approximately 1 second
    // Outtake on
    cargo_Outtake_Time = cargo_Outtake_Time + 1; 
  }
  else { // Done dropping cargo into hub
    // Outtake off
    phase = 2;
    cargo_Outtake_Time = 0;
    cargo_Intake_Time = 0;
  }
}

// Phase 2: Drive to edge of tarmac
if (phase == 2) {
  if (currentRed < 0.286) {  // If current blue value is within 150 (this range can be tested) of past blue value, we are assuming that the surface being sensed has not changed
    m_robotDrive.ArcadeDrive(0, -0.50); // Continue driving until tarmac outline reached; below we would align robot if we are doing that (maybe)
  } 
  else { // Tarmac outlne reached (blue tape sensed) 
    m_encoder1.Reset();
    m_encoder2.Reset();
    encoderAverage=0;
    phase = 3;
 // Prepare for phase 3
  }
} 

if (phase == 3) {
  if (m_encoder1.GetDistance() < 15) { // Real distance = 350/8 inches
    m_robotDrive.ArcadeDrive(0, -0.50);
    // if (encoderAverage < 10) {
    //   closetoCargo=true;
    // }
  }
  // if (cargo_Intake_Time < 100) { 
  //   // Intake on
  //   cargo_Intake_Time++; 
  //   m_robotDrive.ArcadeDrive(0, -0.50);
  else if (reset==false) { // Done picking up cargo
    // Intake off
    phase = 0;
    reset = true;
    cargo_Intake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Back to phase 0
  }
  else {
    phase = 4;
  }
}


 // Route 3
 if(phase3==0){//Outtake ball
      if (cargo_Outtake_Time < 150) { 
        // Outtake on
        cargo_Outtake_Time++;
      }else{
        phase3++;
      }
  }
  else if(phase3==1){//Go forwards til sense blue
    if(currentBlue < 0.300) {
      m_robotDrive.ArcadeDrive(0,-0.5);
    } 
    else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
      m_gyro.Reset();

    }
  }
  else if(phase3==2){//Turn X degrees
    if(abs(m_gyro.GetAngle())<60){
      m_robotDrive.ArcadeDrive(-0.6,0);
    } else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
    }
  }
  else if(phase3==3){//Go forwards 20 inches, and intake is on for last 10 inches 
    if(m_encoder1.GetDistance()<=10){
      m_robotDrive.ArcadeDrive(0,-0.5);
    }else if(m_encoder1.GetDistance()<20){
      //Turn on intake
      m_robotDrive.ArcadeDrive(0,-0.5);
    } else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
    }
  }
  else if(phase3==4){//Go backwards till sense blue
    if(currentBlue < 0.300) {
      m_robotDrive.ArcadeDrive(0,0.5);
    }else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
      m_gyro.Reset();
    }
  }
  else if(phase3==5){ // Turn X degrees
    if(abs(m_gyro.GetAngle())<60){
      m_robotDrive.ArcadeDrive(0.6,0);
    } else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
    }
  }
  else if(phase3==6){ //Go backwards until you are 12 inches from wall, using ultrasonic, ultrasonic bit buggy socomment out
    if(ultrasonic_sensor_range_one>15){
      m_robotDrive.ArcadeDrive(0,0.5);
    } else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
    }
  }
  else if(phase3==6){ //6 inches drive backwards to hub
    if(m_encoder1.GetDistance()>-12){
      m_robotDrive.ArcadeDrive(0,0.5);
    } else{
      phase3++;
      m_encoder1.Reset();
      m_encoder2.Reset();
    }
  }
  else if(phase3==7){ //Outtake ball
    if (cargo_Outtake_Time < 150) { 
      // Outtake on
      cargo_Outtake_Time++;
    }else{
      phase3++;
    }
  }


// Route 4

if (phase4==0){ //Turn 180 degrees
    if(m_gyro.GetAngle()>-180){
        m_robotDrive.ArcadeDrive(0.6,0);
      }
      else{
        phase4=1;
        m_encoder1.Reset();
        m_encoder2.Reset();
      }
    }
    if (phase4==1) //Go back until 40 inches away from wall
    {
      if (ultrasonic_sensor_range_one>=20.0){
        m_robotDrive.ArcadeDrive(0, 0.5); // Drive until 40 inches from back wall
        m_encoder1.Reset();
        m_encoder2.Reset();
      }else{
        phase4=2;
       m_gyro.Reset();

      }
    }
  if (phase4==2){ //Turn 90 degrees
      if(abs(m_gyro.GetAngle())<90){
        m_robotDrive.ArcadeDrive(0.6,0);
      }else{
        phase4=3;
        m_encoder1.Reset();
        m_encoder2.Reset();
      }
    }
    //Go fowards till terminal ball and push it to the human player
    if (phase4==3){
      if(m_encoder2.GetDistance()>=-20){
        m_robotDrive.ArcadeDrive(0,0.5);
      } else{
        if (cargo_Intake_Time < 150) { 
          // Intake on
          cargo_Intake_Time++; 
          m_robotDrive.ArcadeDrive(0, 0.5);
          phase4=4;
        }
      }
    //That is it???
    }
} 

void Robot::TeleopInit() {
  m_encoder1.Reset();
  m_encoder2.Reset();
  m_gyro.Reset();
}

void Robot::TeleopPeriodic() {
  // IntakeMovement();
  // IntakeOnAndOff();
  Storage();
  Outake();
  Movement();
  // Hanging1();
  // Hanging2();
  // Hanging3();
  SmartDashboard();
}

void Robot::Movement() {
  // Drive with arcade style
  float xDrive = m_xbox.GetX() * -0.8;
  float yDrive = m_xbox.GetY() * 0.8;
  m_robotDrive.ArcadeDrive(xDrive, yDrive);
}

void Robot::SmartDashboard() {
  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder 1 Distance: ", m_encoder1.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder 2 Distance: ", m_encoder2.GetDistance());
  frc::SmartDashboard::PutNumber("NAV sensor", abs(m_gyro.GetAngle()));
  frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);
  frc::Color detectedColor = m_colorSensor.GetColor();  
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
  double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  double  ultrasonic_sensor_range_two = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);
  frc::SmartDashboard::PutNumber("Sensor 2 Range", ultrasonic_sensor_range_two);
  // frc::SmartDashboard::PutData("Field", &m_field);
}

void Robot::Storage() {
  m_storage.Set(m_xbox.GetRawAxis(3));
}

void Robot::Outake() {
  m_shooter1.Set(m_xbox.GetRawAxis(1));
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