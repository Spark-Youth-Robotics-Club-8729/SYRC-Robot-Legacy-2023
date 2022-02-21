// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "AHRS.h"
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
#include "cameraserver/CameraServer.h"
#include "frc/motorcontrol/PWMVictorSPX.h"
#include <frc/SpeedControllerGroup.h>
#include "ctre/Phoenix.h"

class Robot : public frc::TimedRobot {

public:
  
  //Gyroscope
  AHRS m_gyro{frc::SPI::Port::kMXP};

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
  static const int leftLeadDeviceID = 0;
  // static const int leftFollowDeviceID = 3;
  static const int rightLeadDeviceID = 1;
  // static const int rightFollowDeviceID = 4;
  static const int Shooter1RioPin = 1;
  static const int Shooter2RioPin = 2;
  static const int IntakeMotorRioPin = 3;



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
  float currentRed;
  float currentBlue;

  

  // RGB Values
  // static constexpr frc::Color kBlueCargo = frc::Color(0, 0, 0);
  // static constexpr frc::Color kRedCargo = frc::Color(0, 0, 0);
  // static constexpr frc::Color kBlueTarmac = frc::Color(0, 0, 0);
  // static constexpr frc::Color kBlackLine = frc::Color(0, 0, 0); //We'll need to detect these on Sunday

  // bool arrivedDestination = false;

  //Joystick
  frc::Joystick m_xbox{ 0 };

  //Drive
  WPI_VictorSPX frontLeft = {leftLeadDeviceID};
  WPI_VictorSPX frontRight = {rightLeadDeviceID};
  // rev::CANSparkMax m_leftLeadMotor{leftLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  // rev::CANSparkMax m_rightLeadMotor{rightLeadDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  // rev::CANSparkMax m_leftFollowMotor{leftFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  // rev::CANSparkMax m_rightFollowMotor{rightFollowDeviceID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DifferentialDrive m_robotDrive{frontLeft, frontRight};

//minecraft

  //Intake/Shooter
  frc::PWMVictorSPX m_Shooter1Motor {Shooter1RioPin};
  frc::PWMVictorSPX m_Shooter2Motor {Shooter2RioPin};
  frc::PWMVictorSPX m_IntakeMotor {IntakeMotorRioPin};
  frc::SpeedControllerGroup m_Shooter {m_Shooter1Motor, m_Shooter2Motor};
  //Encoder Set Up
  frc::Encoder m_encoder1{ EncoderPin1A, EncoderPin1B, true };
  frc::Encoder m_encoder2{ EncoderPin2A, EncoderPin2B, false };
  float encoderAverage;

  // Ultrasonic Set Up
  frc::AnalogInput ultrasonic_sensor_one{0};
  frc::DigitalOutput ultrasonic_trigger_pin_one{4};
  double ultrasonic_sensor_range_one = 0.0;
  
  // frc::AnalogInput ultrasonic_sensor_two{0};
  // frc::DigitalOutput ultrasonic_trigger_pin_two{4};
  // double ultrasonic_sensor_range_two = 0.0;

  double voltage_scale_factor = 1.0;

  // Autonomous Set Up
  int counter;
  int phase;
  int phase4;
  int phase3;
  int cargo_Outtake_Time;
  int cargo_Intake_Time;
  bool distance;
  bool reset;
  bool closetoCargo;  
  int increment;
  bool SenseColour;
  // bool once;

  //Default
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
