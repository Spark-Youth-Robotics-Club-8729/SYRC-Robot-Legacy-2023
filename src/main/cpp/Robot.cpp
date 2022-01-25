// XD Copyright (c) FIRST and other WPILib contributors.
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


  double voltage_scale_factor = 5/frc::RobotController::GetVoltage5V();
  double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.125 * voltage_scale_factor;
  frc::SmartDashboard::PutNumber("Sensor 1 Range", ultrasonic_sensor_range_one);


}

void Robot::AutonomousInit() 
{

m_encoder1.Reset();
m_encoder2.Reset();
ultrasonic_trigger_pin_one.Set(true);
distance = true;
once = true;
pastBlue = 0.0;
currentBlue = 0.0;

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

m_robotDrive.ArcadeDrive(1,0);
  frc::Color detectedColor = m_colorSensor.GetColor();  
  std::string colorString;
  double confidence = 0.0;
  frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence); //Detecting the colour our sensor sees

  // Ultrasonic
  //double  ultrasonic_sensor_range_one = ultrasonic_sensor_one.GetValue() * 0.0492 * voltage_scale_factor;
  // if (ultrasonic_sensor_range_one > 10) {
  //   m_robotDrive.ArcadeDrive(1, 0);
  // } else {
  //   m_robotDrive.ArcadeDrive(0, 0.5);
  // }
  

  // Route 3
  // if (ultrasonic_sensor_range_one>5) { 
  //   m_robotDrive.ArcadeDrive(0.75, 0);
  // } else {
  //   m_robotDrive.ArcadeDrive(0, 0);
  //   // add motor to shoot
  // }
  while (matchedColor == kBlueTarmac) {
    m_robotDrive.ArcadeDrive(0.8, 0);
  }


  


// Route 1: 
// 1. Drop the cargo ball (already stored) into lower hub
// 2. Move forward into the cargo 
  // Sensor use:
    // Sense end of tarmac with colour sensor
    // Use 40 3/8 in to get to Cargo ball
// 3. Pickup Cargo
  // Sensor use:
    // Sense the cargo ball with our ultrasonic sensor
    // Drive until our encoders get that value
// 4. Drive back
// 5. Outtake into lower hub
// 6. Drive forward to get out of tarmac
  
  //This is code assuming we DON'T start next to the hub. Will keep for testing purposes if you delete !kick
  if (ultrasonic_sensor_range_one > 12 && distance == true) { //Change 5 to set distance //Start of 1. and 4. Checking if ultrasonic in range
    m_robotDrive.ArcadeDrive(-0.75, 0);
  } else 
  {
    distance = false;
    m_encoder1.Reset();
    m_encoder2.Reset();
    if ((m_encoder1.GetDistance() + m_encoder2.GetDistance())/2 > -6) //Ultrasonic can't be too precise, so distance needs to go rest of remaining distance
    {
      m_robotDrive.ArcadeDrive (-0.75, 0); 
    } 
    else 
    {
      //Shoot/Drop ball here
    }
  } 
  if (currentBlue < pastBlue + 150) //Start of 2. //Need to test "150" number. TO CLARIFY: Current blue is the current color sensed, and past blue is the one sensed 20ms
                                    // ago (iteration period). +150 is colour not distance
  {
    m_robotDrive.ArcadeDrive(0.75, 0); //Below we would allign robot if we are doing that (maybe)
    m_encoder1.Reset();
  } 
  else 
  {
    if (m_encoder1.GetDistance() < 350/8 && once == true) //Distance is 323/8 -- Should be less/more (test) than this number. This if is to go to cargo ball. 
                                                          //Only enters once colour sensor out of tarmac. Only enters once bc we only pick up ball once :glasses:
                                                        
    {
      if (m_encoder1.GetDistance() > 275/8) //You are at cargo pick it up bozo
      {
        //Turn on Intake to get ball in robot :skull:
      }
    m_robotDrive.ArcadeDrive(0.5, 0);
    //End of 2.   // add intake motor to intake
      distance = true; 
      once = false;
    } 
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
            // m_robotDrive.ArcadeDrive(0, 0); //Take in values for ArcadeDrive(turning, -(going forward))
//         } 
//     }


    //ROUTE 2
    double halfFromHubToCargo = 123.25;
    int angleTurn=0; //To be determined
  if (ultrasonic_sensor_range_one > 12 && distance == true) { //Change 5 to set distance //Start of 1. and 4.
    m_robotDrive.ArcadeDrive(-0.75, 0);
    //Shoot ball
  } else
  {
    distance = false;
    if (m_encoder1.GetDistance()< 5){
      m_robotDrive.ArcadeDrive(-0.5, 0);
    } 
  }
  if (matchedColor == kBlueTarmac) //Start of 2.
  {
    m_robotDrive.ArcadeDrive(0.75, 0);
    m_encoder1.Reset();
  } else if (m_encoder1.GetDistance() < 360/8 && once == true) //Should be less than this number
  {
    m_robotDrive.ArcadeDrive(0.5, 0);
    if (m_encoder1.GetDistance() >275/8){
      printf("Run Intake");
      // Intake code
    }
  //End of 2.   // add intake motor to intake
    distance = true; 
    once = false;
  } else if (ultrasonic_sensor_range_one > 5)
  {
    m_robotDrive.ArcadeDrive (0.75, 0);
  }
  
}

void Robot::TeleopInit() {
  m_encoder1.Reset();
  m_encoder2.Reset();
  
}

void Robot::TeleopPeriodic() {

  // Drive with arcade style
  float xDrive = m_xbox.GetX() * 0.8;
  float yDrive = m_xbox.GetY() * -0.8;
  m_robotDrive.ArcadeDrive(yDrive, xDrive);

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