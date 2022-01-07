// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#if 0
#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <iostream>

#include "CustomPID.h"

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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
#endif

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include "frc/DigitalInput.h"
#include <iostream>
#include <string>
#include <wpi/Format.h>
#include "CustomPID.h"

class Robot : public frc::TimedRobot {

public:
    // default functions
    void RobotInit() override;
    void RobotPeriodic() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TestPeriodic() override;

private:
    // RIO pin constants
    const int RIGHT_FRONTMOTOR_PIN = 1;
    const int RIGHT_BACKMOTOR_PIN = 2;
    const int LEFT_FRONTMOTOR_PIN = 3;
    const int LEFT_BACKMOTOR_PIN = 0;
    const int INTAKE_MOTOR_PIN = 9;
    const int HATCH_MOTOR_PIN = 5;
    const int INTAKE_CARGO_PIN = 2;
    const int ARM_ENCODER_A_PIN = 0;
    const int ARM_ENCODER_B_PIN = 1;
    const int ARM_ENCODER2_A_PIN = 2;
    const int ARM_ENCODER2_B_PIN = 3;




    // PCM pin constants
    const int INTAKE_SOLONOID_PIN_1 = 0;
    const int INTAKE_SOLONOID_PIN_2 = 1;
    const int HATCH_SOLONOID_PIN_1 = 2;
    const int HATCH_SOLONOID_PIN_2 = 3;

    // camera constants
    const int CAMERA_RES_W = 320;
    const int CAMERA_RES_H = 240;
    const int CAMERA_FPS = 15;
    // logitech constants
    const int COMPRESSOR_ON_BUTTON = 1;
    const int COMPRESSOR_OFF_BUTTON = 3;
    const int SOLENOID_ON = 2;
    const int SOLENOID_OFF = 4;
    const int INTAKE_EX = 6;
    const int INTAKE_RE = 5;
    const int INTAKE_OFF = 9;
    const int HATCH_FORW = 7;
    const int HATCH_BACK = 8;
    const int HATCH_OFF = 10;

    // x-box controller constants
    
    const int ARM_HEIGHT_1_BUTTON = 1;
    const int ARM_HEIGHT_2_BUTTON = 2;
    
    const int ARM_HEIGHT_3_BUTTON = 3;
    const int ARM_HEIGHT_4_BUTTON = 4;
    const int MANUAL_OVERRIDE_TRIGGER = 2;
    const int MANUAL_LIFT_AXIS = 1;
    const int INTAKE_AXIS = 5;
    const int HATCH_AXIS = 6;
    // speed constants
    const float INTAKE_SPEED = -1.0;
    const float HATCH_SPEED = -1.0;
    const float ARM_SPEED = 1.0;
    const float DRIVE_X_SPEED = 0.8;
    const float DRIVE_Y_SPEED = -0.7;
    // arm height constants
    const float ARM_HEIGHT_1 = 0;
    const float ARM_HEIGHT_2 = -275;
    const float ARM_HEIGHT_3 = -350;
    const float ARM_HEIGHT_4 = -475;

    // encoder
    double distance = 0.0;
    double distance2 = 0.0;
    bool arrivedDestination;

    //manual override
    bool manualOverride = false;

    // joystick
    frc::Joystick m_stick{ 0 };
    frc::Joystick m_xbox{ 1 };
    // camera
    cs::UsbCamera m_camera;

    // Digital Input Variable for Limit Switch daniel_code start
    // DigitalInput forwardLimitSwitch, reverseLimitSwitch;
    // daniel_code end

#if 1
    // drive
    frc::PWMVictorSPX m_frontleft{LEFT_FRONTMOTOR_PIN};
    frc::PWMVictorSPX m_frontright{RIGHT_FRONTMOTOR_PIN};
    frc::PWMVictorSPX m_backright{RIGHT_BACKMOTOR_PIN};
    frc::PWMVictorSPX m_backleft{LEFT_BACKMOTOR_PIN};

    frc::SpeedControllerGroup m_right{m_frontright,m_backright};
    frc::SpeedControllerGroup m_left{m_frontleft,m_backleft};
    
    frc::DifferentialDrive m_robotDrive{m_left, m_right};
#endif
    // intake
    frc::PWMVictorSPX m_intakeMotor{ INTAKE_MOTOR_PIN };
    frc::PWMVictorSPX m_hatchMotor{ HATCH_MOTOR_PIN };
    WPI_VictorSPX m_armMotor{ 0 };
    WPI_VictorSPX m_armMotor2{ 1 };
//    frc::DigitalInput m_cargoSwitch {INTAKE_CARGO_PIN};
    // pneumatics
    frc::Compressor* m_compressor = new frc::Compressor(0);

    frc::DoubleSolenoid m_intakeSolenoid{ INTAKE_SOLONOID_PIN_1, INTAKE_SOLONOID_PIN_2 };
#if 0
    frc::DoubleSolenoid m_hatchSolenoid{ HATCH_SOLONOID_PIN_1, HATCH_SOLONOID_PIN_2 };
#endif
    // encoder
    CustomPID m_encoderPID;
    //frc::Encoder m_liftEncoder{ ARM_ENCODER_A_PIN, ARM_ENCODER_B_PIN, true };
    //frc::Encoder m_liftEncoder2{ ARM_ENCODER2_A_PIN, ARM_ENCODER2_B_PIN, false };
    frc::Encoder m_drivetrainEncoder1{ ARM_ENCODER_A_PIN, ARM_ENCODER_B_PIN, true };
    frc::Encoder m_drivetrainEncoder2{ ARM_ENCODER2_A_PIN, ARM_ENCODER2_B_PIN, false };


    // periodic functions
    void ToggleManualOverride();
#if 0
    void ControlHatchPiston();
#endif
    void DriveWithJoystick();
    void ControlIntakePiston();
    void ControlCompressorEnabledState();
    void ControlIntakeMotor();
    void ControlHatchMotor();
    void SetArmTargetAngle();
    void ControlArmMotor();
    void AutoControlArmMotor();
    void ManualControlArmMotor();
    void DisplayShuffleBoardInformation();
};

#if 0
#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

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
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
#endif
