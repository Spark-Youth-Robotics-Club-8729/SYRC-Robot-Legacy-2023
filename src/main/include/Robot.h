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
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include "frc/PneumaticsBase.h"
#include "frc/PneumaticsModuleType.h"
#include <frc/DoubleSolenoid.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Compressor.h>
#include <frc/motorcontrol/MotorControllerGroup.h>


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
  static const int storageID = 1;
  static const int shooterID = 2;
  static const int intakeDeviceID = 3;
  static const int leftLeadDeviceID = 4;
  static const int rightLeadDeviceID = 5;
  static const int leftBackDeviceID = 6;
  static const int rightBackDeviceID = 7;
  static const int Hanger1ID = 8; 
  static const int Hanger2ID = 9; 
  static const int Hanger3ID = 10; 
  static const int Hanger4ID = 11;
  static const int Hanger5ID = 12;
  static const int Hanger6ID = 13;



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
  

  // Pneumatics
  static const int Pneumatics1 = 1;
  static const int Pneumatics2 = 0;
  frc::DoubleSolenoid m_pneumatics{frc::PneumaticsModuleType::CTREPCM, Pneumatics1, Pneumatics2};
  frc::Compressor pcmCompressor {0, frc::PneumaticsModuleType::CTREPCM};

  //Gyro
  AHRS m_gyro{frc::SPI::Port::kMXP};

  //Joystick
  frc::Joystick m_xbox{ 0 }; //MAKE SURE IN DRIVERSTATION CONTROLLER IS ON 0.
  frc::Joystick m_stick{ 1 }; //MAKE SURE IN DRIVERSTATION CONTROLLER IS ON 1.

  //Hanging
  WPI_VictorSPX InnerLeftClimber = {Hanger1ID};
  WPI_VictorSPX InnerRightClimber = {Hanger2ID};
  WPI_VictorSPX OuterLeftClimber = {Hanger3ID};
  WPI_VictorSPX OuterRightClimber = {Hanger4ID};
  WPI_VictorSPX InnerClimberLateral = {Hanger5ID};
  WPI_VictorSPX OuterClimberLateral = {Hanger6ID};


  //DifferentialDrive
  WPI_VictorSPX frontLeft = {leftLeadDeviceID};
  WPI_VictorSPX frontRight = {rightLeadDeviceID};
  WPI_VictorSPX backRight = {rightBackDeviceID};
  WPI_VictorSPX backLeft = {leftBackDeviceID};
  frc::MotorControllerGroup m_left{frontLeft, backLeft};
  frc::MotorControllerGroup m_right{frontRight, backRight};
  frc::DifferentialDrive m_robotDrive{m_left, m_right};
  bool reverse = false;

  //Intake
  rev::CANSparkMax intake {intakeDeviceID, rev::CANSparkMax::MotorType::kBrushed};

  //Storage/Shooter
  rev::CANSparkMax m_shooter{shooterID, rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax m_storage{storageID, rev::CANSparkMax::MotorType::kBrushless};

  //Camera
  double targetOffsetAngle_Horizontal = 0.0;
  double targetOffsetAngle_Vertical = 0.0;
  double targetArea = 0.0;
  bool intaked = false;
  const int Camera_Button = 9;

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
  void Camera();
  void Intake();
  void Storage();
  void Outtake();
  void Movement();
  void RMovement();
  void Hanging1();
  void SmartDashboard();
  void Pneumatics();

  //Default
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};