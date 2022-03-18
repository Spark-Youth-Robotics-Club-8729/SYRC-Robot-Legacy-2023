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
  camera.SetResolution (640, 480);
  
  //Encoders
  m_encoder1.Reset();
  m_encoder2.Reset();
  m_encoder1.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
  m_encoder2.SetDistancePerPulse((3.14159265358 * 6) / 360.0);


  // Ultrasonic 
  ultrasonic_trigger_pin_one.Set(true);


  //Default
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {

//Reset/Initializing Sensors
m_encoder1.Reset();
m_encoder2.Reset();
ultrasonic_trigger_pin_one.Set(true);

//Initializing Variables
currentRed = 0.0;
currentBlue = 0.0;
cargo_Outtake_Time = 0;
cargo_Intake_Time = 0;
phase=0;
phase3=0;
phase4=0;
encoderAverage=0.0;
reset=false;

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
frc::SmartDashboard::PutNumber("NAV sensor", (m_gyro.GetYaw()));


//Encoder Periodic
encoderAverage = (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2;



//********************************************************************************************************************************


//ROUTE 1:

// Phase 0: Drive from starting position to hub
if (m_autoSelected == kAutoNameCustom) {
if (phase == 0) { 
  
  if (ultrasonic_sensor_range_one > 20) { //If farther than 20 inches from lower hub, driving until 20 inchs from lower hub
    
    m_robotDrive.ArcadeDrive(0, 0.50); 
    m_encoder1.Reset();
    m_encoder2.Reset();

  } 

  else { 
    
    phase = 1;

  }

}

// Phase 1: Outtake ball

if (phase == 1) {

    if (cargo_Outtake_Time <= 200){
    // Outtake on here
    cargo_Outtake_Time+=1;
    m_shooter.Set(0.9);
      if (cargo_Outtake_Time >= 50){
        m_storage.Set(0.9);
      }
    }

  else { //Outtake off, and onto the next phase
    
    phase = 2;
    cargo_Outtake_Time = 0;
    cargo_Intake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

// Phase 2: Drive to edge of tarmac OR getting out of tarmac when reset happens

if (phase == 2 && reset==false) {

  if (currentRed < 0.286) {  // Driving until a high sense in colour, Depends on whether we are Red or Blue
    
    m_robotDrive.ArcadeDrive(0, -0.50); 

  } 
  
  else { //Reached Tarmac, onto phase 3
    
    m_encoder1.Reset();
    m_encoder2.Reset();
    encoderAverage=0;
    phase = 3;

  }

}

if (phase == 2 && reset==true) { //Exiting Tarmac

  if (encoderAverage < 50) {

    m_robotDrive.ArcadeDrive(0, -0.50);

  }

  else { //End of Route 1

    phase = 4;

  }
  
} 

if (phase == 3) {
  
  if (m_encoder1.GetDistance() < 15) { //Real distance = 350/8 inches, Driving until we reach the cargo 
    
    m_robotDrive.ArcadeDrive(0, -0.50);
    //Intake comes here
    if (m_encoder1.GetDistance() > 10 ) { 
      intake.Set(0.9);
    }
  }
  else if (reset==false) { // Done picking up cargo
    
    phase = 0;
    reset = true;
    cargo_Intake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Back to phase 0

  }

  else {

    phase = 4;
    m_robotDrive.ArcadeDrive(0,0);
  }

}

}
//********************************************************************************************************************************


// Route 3:
else {
if(phase3==0) { //Outtake ball
    
    if (cargo_Outtake_Time <= 200){
    // Outtake on here
    cargo_Outtake_Time+=1;
    m_shooter.Set(0.9);
      if (cargo_Outtake_Time >= 50){
        m_storage.Set(0.9);
      }
    }
    
  else {
        
      phase3++;

    }

  }
else if(phase3==1) {//Go forwards until we sense blue
  
  if(currentBlue < 0.286) {

    m_robotDrive.ArcadeDrive(0,-0.5);

  } 
  
  else {

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();
    m_gyro.Reset();

  }

}

else if(phase3==2) { //Turn 60 degrees
  
  if(abs(m_gyro.GetYaw())<60){

    m_robotDrive.ArcadeDrive(-0.6,0);

  } 

  else {

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

else if (phase3==3) {//Go forwards 20 inches, and intake is on for last 10 inches 

  if(m_encoder1.GetDistance()<20){ 
    m_robotDrive.ArcadeDrive(0,-0.5);

    if(m_encoder1.GetDistance()>=10){
      intake.Set(0.9); //Intake on
    
    } 
  }
  else{

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

else if(phase3==4){//Go backwards till sense blue
  
  if(currentBlue < 0.300) {

    m_robotDrive.ArcadeDrive(0,0.5);

  }
  else{

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();
    m_gyro.Reset();

  }

}

else if (phase3==5) { // Turn X degrees
  
  if(m_gyro.GetYaw()<60){

    m_robotDrive.ArcadeDrive(0.6,0);

  } 
  else{

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

else if (phase3==6) { //Go backwards until you are 12 inches from wall, using ultrasonic, ultrasonic bit buggy socomment out
  
  if(ultrasonic_sensor_range_one>12){

    m_robotDrive.ArcadeDrive(0,0.5);

  } 
  else{

    phase3++; 
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

else if (phase3==6) { //6 inches drive backwards to hub
  
  if(m_encoder1.GetDistance()>-6){

    m_robotDrive.ArcadeDrive(0,0.5);

  } 
  else{

    phase3++;
    m_encoder1.Reset();
    m_encoder2.Reset();

  }

}

else if (phase3==7) { //Outtake ball
    if (cargo_Outtake_Time <= 200){
    // Outtake on here
    cargo_Outtake_Time+=1;
    m_shooter.Set(0.9);
      if (cargo_Outtake_Time >= 50){
        m_storage.Set(0.9);
      }
    }
  else {

    phase3++;

  }

}

}
//********************************************************************************************************************************


// // Route 4:
// if (phase4==-1){//Outtake ball
//     if (cargo_Outtake_Time <= 200){ 
    
//     cargo_Outtake_Time+=1; 
//     m_shooter.Set(0.9);
//       if (cargo_Outtake_Time >= 50){
//         m_storage.Set(0.9); 
//       } 
//     }
//     else{
//       phase4=0;
//     }
// }
// if (phase4==0){ //Turn 180 degrees

//   if(m_gyro.GetYaw()>-180){

//     m_robotDrive.ArcadeDrive(0.6,0);

//   }
  
//   else {

//     phase4=1;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// if (phase4==1) { //Go back until 40 inches away from wall

//   if (ultrasonic_sensor_range_one>=20.0) {

//     m_robotDrive.ArcadeDrive(0, 0.5); // Drive until 40 inches from back wall
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }
  
//   else {

//     phase4=2;
//     m_gyro.Reset();

//   }

// }

// if (phase4==2){ //Turn 90 degrees
  
//   if(m_gyro.GetYaw()<90) {

//     m_robotDrive.ArcadeDrive(0.6,0);

//   }
  
//   else {

//     phase4=3;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// if (phase4==3) { //Go fowards till terminal ball and push it to the human player
  
//   if (m_encoder2.GetDistance()>=-20) {

//     m_robotDrive.ArcadeDrive(0,-0.5);
//     if(m_encoder2.GetDistance()>=-10){
//       intake.Set(0.9);
//     }
//   } 
//   else{
//     phase4++;
//   }

// }
// if (phase4==4){
//   if(m_encoder2.GetDistance()<0){
//     m_robotDrive.ArcadeDrive(0,-0,5);
//   } else{
//     phase4++;
//     m_encoder1.Reset();
//   }
// }
// if (phase4==5){
//     if(m_gyro.GetYaw()>-90) {
//       m_robotDrive.ArcadeDrive(-0.6,0);
//     } 
  
//   else {

//     phase4++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

} 

void Robot::TeleopInit() {

  m_encoder1.Reset();
  m_encoder2.Reset();
  m_gyro.Reset();

}

void Robot::TeleopPeriodic() {

  Intake();
  Storage();
  Outtake();
  Movement();
  RMovement();
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

}

void Robot::Hanging1() { 

InnerLeftClimber.Set(m_xbox.GetRawAxis(1)*0.75);
InnerRightClimber.Set(m_xbox.GetRawAxis(1)*0.75);
OuterLeftClimber.Set(m_xbox.GetRawAxis(3)*0.75);
OuterRightClimber.Set(m_xbox.GetRawAxis(3)*0.75);
InnerClimberLateral.Set(m_xbox.GetRawAxis(0)*0.80);
OuterClimberLateral.Set(m_xbox.GetRawAxis(2)*0.80);

// if (m_stick.GetRawButton(7)) {

// InnerLeftClimber.Set(0.4);
// InnerRightClimber.Set(0.4);

// }

// if (m_stick.GetRawButton(8)) {

// OuterLeftClimber.Set(0.4);
// OuterRightClimber.Set(0.4);

// }


}

void Robot::Movement() {

  // Drive with arcade style
  float xDrive = m_stick.GetX() * 0.8;
  float yDrive = m_stick.GetY() * -0.8;
  m_robotDrive.ArcadeDrive(xDrive, yDrive);

}

void Robot::RMovement() {

  if (m_stick.GetRawButton(2)) {

  // Drive with arcade style
  float xDrive = m_stick.GetX() * 0.8;
  float yDrive = m_stick.GetY() * 0.8; 
  m_robotDrive.ArcadeDrive(xDrive, yDrive);

  }

}

void Robot::SmartDashboard() {

  // Encoder SmartDashboard
  frc::SmartDashboard::PutNumber("Encoder 1 Distance: ", m_encoder1.GetDistance());
  frc::SmartDashboard::PutNumber("Encoder 2 Distance: ", m_encoder2.GetDistance());
  frc::SmartDashboard::PutNumber("Robot Displacement: ", (m_encoder1.GetDistance() + m_encoder2.GetDistance())/2);

  //Gyro SmartDashboard
  frc::SmartDashboard::PutNumber("NAV sensor", m_gyro.GetYaw());

  //Colour Sensor SmartDashboard
  frc::Color detectedColor = m_colorSensor.GetColor();  
  frc::SmartDashboard::PutNumber("Red", detectedColor.red);
  frc::SmartDashboard::PutNumber("Green", detectedColor.green);
  frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);

  //Ultrasonic SmartDashboard
  double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);

  //Limelight
  std::shared_ptr table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  targetArea = table->GetNumber("ta",0.0);

}

void Robot::Storage() {

  if (m_xbox.GetRawButton(5)) {

    m_storage.Set(-0.80);

  } 

  if (m_xbox.GetRawButton(6)) { 

    m_storage.Set(0.0);

  }

}

void Robot::Outtake() {

  if (m_xbox.GetRawButton(7)) {

    m_shooter.Set(0.80);

  } 

  if (m_xbox.GetRawButton(8)) { 

    m_shooter.Set(0.0);

  }

}

void Robot::Camera() {

  // if (m_stick.GetRawButton(9) && intaked == true) {

  //   intaked = false;

  //   while ( targetOffsetAngle_Horizontal < -5 || targetOffsetAngle_Horizontal > 5) {

  //     if ( targetOffsetAngle_Horizontal < -5) {

  //       m_robotDrive.ArcadeDrive(0, 0.5);

  //     }

  //     else if ( targetOffsetAngle_Horizontal > 5) {

  //       m_robotDrive.ArcadeDrive(0, -0.5);

  //     }

  //   }

  //   m_encoder1.Reset();
  //   m_encoder2.Reset();

  //   while (intaked == false) {

  //   if ((m_encoder2.GetDistance() + m_encoder1.GetDistance())/2 < 60)  { //Test max distance away from cam
      
  //     m_robotDrive.ArcadeDrive(0.6, 0);
  //     intake.Set(0.8);

  //     if (m_xbox.GetRawButton(9) && intaked == false) {

  //       intaked = true;

  //     }

  //   }

  //   }

  // }

}

void Robot::Pneumatics() {
  if (m_xbox.GetRawButton(9)) {
    pcmCompressor.Start();
  }
  if (m_xbox.GetRawButton(10)) {
    pcmCompressor.Stop();
  }
  if (m_xbox.GetRawButton(2)) {
    m_pneumatics.Set(frc::DoubleSolenoid::Value::kForward);
  }
  if (m_xbox.GetRawButton(4)) {
    m_pneumatics.Set(frc::DoubleSolenoid::Value::kReverse);
  }
  else {
    m_pneumatics.Set(frc::DoubleSolenoid::Value::kOff);
  }
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