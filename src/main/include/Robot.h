// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalOutput.h>
#include "frc/RobotController.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/util/color.h>
#include <frc/AnalogGyro.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>




class Robot : public frc::TimedRobot {

public:

  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

private:

  //CAN Pin Constants
  static const int leftLeadDeviceID = 2;
  static const int leftFollowDeviceID = 3;
  static const int rightLeadDeviceID = 1;
  static const int rightFollowDeviceID = 4;

  //Encoder Pin Constants
  static const int EncoderPin1A = 0;
  static const int EncoderPin1B = 1;
  static const int EncoderPin2A = 2;
  static const int EncoderPin2B = 3;

  // Field2d
  // frc::Field2d m_field;

  // I2C Port
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  // Colour Sensor
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  float currentBlue;
  float pastBlue;

  //Gryo
  // frc::AnalogGyro m_gyro{0};

  // RGB Values
  static constexpr frc::Color kBlueCargo = frc::Color(0, 0, 0);
  static constexpr frc::Color kRedCargo = frc::Color(0, 0, 0);
  static constexpr frc::Color kBlueTarmac = frc::Color(0, 0, 0);
  static constexpr frc::Color kBlackLine = frc::Color(0, 0, 0); //We'll need to detect these on Sunday

  // bool arrivedDestination = false;

  //Joystick
  frc::Joystick m_xbox{ 0 };

  //Drive
  rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive m_robotDrive{m_leftLeadMotor, m_rightLeadMotor};

  //Encoder Set Up
  frc::Encoder m_encoder1{ EncoderPin1A, EncoderPin1B, true };
  frc::Encoder m_encoder2{ EncoderPin2A, EncoderPin2B, false };
  float encoderAverage;

  // Ultrasonic Set Up
  frc::AnalogInput ultrasonic_sensor_one{0};
  frc::DigitalOutput ultrasonic_trigger_pin_one{4};
  double ultrasonic_sensor_range_one = 0.0;
  double voltage_scale_factor = 1.0;

  // Autonomous Set Up
  int counter;
  int phase;
  int phase4;
  int cargo_Outtake_Time;
  int cargo_Intake_Time;
  bool distance;
  bool reset;
  bool closetoCargo;
  // bool once;

  //Default
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};