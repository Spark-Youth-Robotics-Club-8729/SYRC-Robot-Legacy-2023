// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

//Libraries
#include "AHRS.h"
#include <string>
#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include "rev/CANSparkMax.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Encoder.h>
#include <frc/AnalogInput.h>
#include <frc/DigitalOutput.h>
#include "frc/RobotController.h"
#include "rev/ColorSensorV3.h"
#include "rev/ColorMatch.h"
#include <frc/util/color.h>
#include <frc/AnalogGyro.h>
#include "cameraserver/CameraServer.h"
#include "frc/motorcontrol/PWMVictorSPX.h"
#include "ctre/Phoenix.h"


class Robot : public frc::TimedRobot {

public:

  //Functions
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
  static const int leftLeadDeviceID = 3;
  static const int rightLeadDeviceID = 4;
  static const int intakeDeviceID = 5;

  static const int storageID = 1;
  static const int shooterID = 2;


  //Encoder Pin Constants
  static const int EncoderPin1A = 0;
  static const int EncoderPin1B = 1;
  static const int EncoderPin2A = 2;
  static const int EncoderPin2B = 3;

  // I2C Port
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;

  // Colour Sensor
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;
  float currentRed;
  float currentBlue;
  
  //Gyro
  AHRS m_gyro{frc::SPI::Port::kMXP};

  //Joystick
  frc::Joystick m_xbox{ 0 }; //MAKE SURE IN DRIVERSTATION CONTROLLER IS ON 0.

  //DifferentialDrive
  WPI_VictorSPX frontLeft = {leftLeadDeviceID};
  WPI_VictorSPX frontRight = {rightLeadDeviceID};
  WPI_VictorSPX intake = {intakeDeviceID};
  frc::DifferentialDrive m_robotDrive{frontLeft, frontRight};

  //Storage/Shooter
  rev::CANSparkMax m_shooter{shooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_storage{storageID, rev::CANSparkMax::MotorType::kBrushless};

  //Encoder Set Up
  frc::Encoder m_encoder1{ EncoderPin1A, EncoderPin1B, true };
  frc::Encoder m_encoder2{ EncoderPin2A, EncoderPin2B, false };
  float encoderAverage;

  // Ultrasonic Set Up
  frc::AnalogInput ultrasonic_sensor_one{0};
  frc::DigitalOutput ultrasonic_trigger_pin_one{4};
  double ultrasonic_sensor_range_one = 0.0;
  double voltage_scale_factor = 1.0;

  // Autonomous Variables
  int phase;
  int phase4;
  int phase3;
  int cargo_Outtake_Time;
  int cargo_Intake_Time;
  bool reset;

  //Teleop Periodic
  void Intake();
  void Storage();
  void Outtake();
  void Movement();
  // void Hanging1();
  // void Hanging2();
  // void Hanging3();
  void SmartDashboard();

  //Default
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};