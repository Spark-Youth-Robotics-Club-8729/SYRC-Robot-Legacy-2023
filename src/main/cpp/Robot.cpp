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
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);

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
phase=1;
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

// Phase 2: Drive to edge of tarmac OR getting out of tarmac when reset happens

if (m_autoSelected==kAutoNameDefault) {
if (phase == 1) {

  if (m_encoder2.GetDistance() > -61) {  // Driving until a high sense in colour, Depends on whether we are Red or Blue
    
    m_robotDrive.ArcadeDrive(0, 0.70); 
    intake.Set(-0.80);
    m_shooter.Set(0.85);
  } 
  
  else { //Reached Tarmac, onto phase 3
    m_encoder1.Reset();
    m_encoder2.Reset();
    encoderAverage=0;
    phase = 2;

  }

}
if (phase==2) {
  if (cargo_Outtake_Time < 50) {
  cargo_Outtake_Time++;
  }
  else {
    phase=3;
  }
}

if (phase == 3) {
    m_storage.Set(-0.95);
    m_robotDrive.ArcadeDrive(0, 0);
    phase = 4;
    cargo_Intake_Time = 0;
    cargo_Outtake_Time = 0;
    m_encoder1.Reset();
    m_encoder2.Reset(); // Back to phase 0

  }
if (phase==4) {
  if (cargo_Outtake_Time <150) {
    cargo_Outtake_Time++;
    intake.Set(0.0);
    m_shooter.Set(0.0);
    m_storage.Set(0.0);
  }
  else {
    phase=5;
  }
}
}

//********************************************************************************************************************************


// Route 3:
// else if (m_autoSelected==kAutoNameCustom1) {
// if(phase3==0) { //Outtake ball
    
//     if (cargo_Outtake_Time <= 100){
//     // Outtake on here
//     cargo_Outtake_Time+=1;
//     m_shooter.Set(0.9);
//       if (cargo_Outtake_Time >= 25){
//         m_storage.Set(0.9);
//       }
//     }
    
//   else {
        
//       phase3++;

//     }

//   }
// else if(phase3==1) {//Go forwards until we sense blue OR red
  
//   if(currentBlue < 0.286 || currentRed < 0.286) {

//     m_robotDrive.ArcadeDrive(0,-0.5);

//   } 
  
//   else {

//     phase3++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();
//     m_gyro.Reset();

//   }

// }

// else if(phase3==2) { //Turn 60 degrees
  
//   if(abs(m_gyro.GetYaw())<60){

//     m_robotDrive.ArcadeDrive(0.6,0);

//   } 

//   else {

//     phase3++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// else if (phase3==3) {//Go forwards 20 inches, and intake is on for last 10 inches 

//   if(m_encoder1.GetDistance()<20){ 
//     m_robotDrive.ArcadeDrive(0,0.5);

//     if(m_encoder1.GetDistance()>=10){
//       intake.Set(0.9); //Intake on
//     } 
//   }
//   else{

//     phase3++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// else if(phase3==4){//Go backwards till sense blue
  
//   if(currentBlue < 0.300) {

//     m_robotDrive.ArcadeDrive(0,-0.5);

//   }
//   else{

//     phase3++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();
//     m_gyro.Reset();

//   }

// }

// else if (phase3==5) { // Turn X degrees
  
//   if(m_gyro.GetYaw()<60){

//     m_robotDrive.ArcadeDrive(-0.6,0);

//   } 
//   else{

//     phase3++;
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// else if (phase3==6) { //Go backwards until you are 12 inches from wall, using ultrasonic, ultrasonic bit buggy socomment out
  
//   if(m_encoder1.GetDistance()>20)//random encoder value 
//   {

//     m_robotDrive.ArcadeDrive(0,-0.5);

//   } 
//   else{

//     phase3+=2; 
//     m_encoder1.Reset();
//     m_encoder2.Reset();

//   }

// }

// else if (phase3==7) { //Outtake ball
//     if (cargo_Outtake_Time <= 200){
//     // Outtake on here
//     cargo_Outtake_Time+=1;
//     m_shooter.Set(0.9);
//       if (cargo_Outtake_Time >= 50){
//         m_storage.Set(0.9);
//       }
//     }
//   else {

//     phase3++;

//   }

// }

// }

// // ********************************************************************************************************************************

// else if (m_autoSelected == kAutoNameCustom2) {
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
//       cargo_Outtake_Time=0;
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

//   if (m_encoder1.GetDistance()>20) //random encoder value 
//   {

//     m_robotDrive.ArcadeDrive(0, -0.5); // Drive until 40 inches from back wall
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

// if (phase4==3) { //Go backwards till terminal ball and push it to the human player
  
//   if (m_encoder2.GetDistance()>=-20) {

//     m_robotDrive.ArcadeDrive(0,-0.5);
//     // if(m_encoder2.GetDistance()>=-10){ Only needed if picking up ball
//     //   intake.Set(0.9);
//     // }
//   } 
//   else{
//     phase4++;
//   }

// }
// //This code only needed if go back to hub
// // if (phase4==4){
// //   if(m_encoder2.GetDistance()<0){
// //     m_robotDrive.ArcadeDrive(0,-0,5);
// //   } else{
// //     phase4++;
// //     m_encoder1.Reset();
// //   }
// // }
// // if (phase4==5){
// //     if(m_gyro.GetYaw()>-90) {
// //       m_robotDrive.ArcadeDrive(-0.6,0);
// //     } 
  
// //   else {

// //     phase4++;
// //     m_encoder1.Reset();
// //     m_encoder2.Reset();

// //   }

// // }

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

  if (m_xbox.GetRawButton(9)) {

    intake.Set(0.80);

  }

}

void Robot::Hanging1() { 

InnerLeftClimber.Set(m_xbox.GetRawAxis(1)*0.75);
InnerRightClimber.Set(m_xbox.GetRawAxis(1)*0.75);
OuterLeftClimber.Set(m_xbox.GetRawAxis(3)*0.75);
OuterRightClimber.Set(m_xbox.GetRawAxis(3)*0.75);
InnerClimberLateral.Set(m_xbox.GetRawAxis(0)*-0.90);
OuterClimberLateral.Set(m_xbox.GetRawAxis(2)*-0.90);

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
  float xDrive = m_stick.GetRawAxis(4) * 0.8;
  float yDrive = m_stick.GetRawAxis(1) * -0.8;
  m_robotDrive.ArcadeDrive(xDrive, yDrive);
  // m_right.Set(m_stick.GetRawAxis(3));
  // m_left.Set(m_stick.GetRawAxis(1));
  // frontLeft.Set(m_test1.GetRawAxis(1));
  // backLeft.Set(m_test1.GetRawAxis(3)); //BACK LEFT NOT WORKING
  // frontRight.Set(m_test.GetRawAxis(1));
  // backRight.Set(m_test.GetRawAxis(3));


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
  frc::SmartDashboard::PutNumber("Shooter RPM", m_ShooterEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Shooter Speed", m_shooter.Get());
  frc::SmartDashboard::PutNumber("Feeder RPM", m_FeederEncoder.GetVelocity());
  frc::SmartDashboard::PutNumber("Feeder Speed", m_storage.Get());


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
    m_storage.Set(-0.95); //0.45 //0.95
  } 

  if (m_xbox.GetRawButton(6)) { 
    m_storage.Set(0.0);
  }

  // m_storage.Set(m_stick.GetRawAxis(3)*0.95);
}

void Robot::Outtake() {

  if (m_xbox.GetRawButton(7)) {
    m_shooter.Set(-0.65);
  }
   //0.60//0.65 

  if (m_xbox.GetRawButton(8)) { 
    m_shooter.Set(-0.4);
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
  // if (m_xbox.GetRawButton(9)) {
  //   pcmCompressor.Start();
  // }
  // if (m_xbox.GetRawButton(10)) {
  //   pcmCompressor.Stop();
  // }
  if (m_xbox.GetRawButton(2)) {
    m_pneumatics.Set(frc::DoubleSolenoid::Value::kForward);
  }
  if (m_xbox.GetRawButton(4)) {
    m_pneumatics.Set(frc::DoubleSolenoid::Value::kReverse);
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