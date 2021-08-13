/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctre/Phoenix.h>
#include <frc/WPILib.h>
#include <iostream>
#include <string>
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
    const int FRONT_LEFT_MOTOR_PIN = 2;
    const int BACK_LEFT_MOTOR_PIN = 1;
    const int FRONT_RIGHT_MOTOR_PIN = 4;
    const int BACK_RIGHT_MOTOR_PIN = 3;

    const int INTAKE_MOTOR_PIN = 0;
    const int INTAKE_CARGO_PIN = 2;
    const int ARM_ENCODER_A_PIN = 0;
    const int ARM_ENCODER_B_PIN = 1;
    // PCM pin constants
    const int INTAKE_SOLONOID_PIN_1 = 0;
    const int INTAKE_SOLONOID_PIN_2 = 1;
    const int HATCH_SOLONOID_PIN_1 = 2;
    const int HATCH_SOLONOID_PIN_2 = 3;
    // camera constants
    const int CAMERA_RES_W = 320;
    const int CAMERA_RES_H = 240;
    const int CAMERA_FPS = 15;
    // joystick constants
    const int COMPRESSOR_ON_BUTTON = 11;
    const int COMPRESSOR_OFF_BUTTON = 12;
    // x-box controller constants
    const int ARM_HEIGHT_1_BUTTON = 1;
    const int ARM_HEIGHT_2_BUTTON = 2;
    const int ARM_HEIGHT_3_BUTTON = 3;
    const int ARM_HEIGHT_4_BUTTON = 4;
    const int MANUAL_OVERRIDE_TRIGGER = 2;
    const int MANUAL_LIFT_AXIS = 1;
    const int INTAKE_AXIS = 5;
    // speed constants
    const float INTAKE_SPEED = -1.0;
    const float ARM_SPEED = 1.0;
    const float DRIVE_X_SPEED = 1.0;
    const float DRIVE_Y_SPEED = -1.0;
    // arm height constants
    const float ARM_HEIGHT_1 = 0;
    const float ARM_HEIGHT_2 = -275;
    const float ARM_HEIGHT_3 = -350;
    const float ARM_HEIGHT_4 = -475;

    //manual override
    bool manualOverride = false;

    // joystick
    frc::Joystick m_stick{ 0 };
    frc::Joystick m_xbox{ 1 };
    // camera
    cs::UsbCamera m_camera;
 
    // drive
    frc::PWMVictorSPX m_frontLeftMotor{FRONT_LEFT_MOTOR_PIN};
    frc::PWMVictorSPX m_backLeftMotor{BACK_LEFT_MOTOR_PIN};
    frc::PWMVictorSPX m_frontRightMotor{FRONT_LEFT_MOTOR_PIN};
    frc::PWMVictorSPX m_backRightMotor{BACK_RIGHT_MOTOR_PIN };
   
    frc::SpeedControllerGroup m_right=frc::SpeedControllerGroup{m_backRightMotor, m_frontRightMotor};
    frc::SpeedControllerGroup m_left=frc::SpeedControllerGroup{m_backLeftMotor, m_frontLeftMotor};
    frc::DifferentialDrive m_robotDrive{m_left, m_right};


    // intake
    frc::Spark m_intakeMotor{ INTAKE_MOTOR_PIN };
    WPI_VictorSPX m_armMotor{ 0 };
    WPI_VictorSPX m_armMotor2{ 1 };
    frc::DigitalInput m_cargoSwitch {INTAKE_CARGO_PIN};
    // pneumatics
    frc::Compressor* m_compressor = new frc::Compressor(0);
    frc::DoubleSolenoid m_intakeSolenoid{ INTAKE_SOLONOID_PIN_1, INTAKE_SOLONOID_PIN_2 };
    frc::DoubleSolenoid m_hatchSolenoid{ HATCH_SOLONOID_PIN_1, HATCH_SOLONOID_PIN_2 };
    // encoder
    CustomPID m_encoderPID;
    frc::Encoder m_liftEncoder{ ARM_ENCODER_A_PIN, ARM_ENCODER_B_PIN, false, frc::Encoder::EncodingType::k2X };

    // periodic functions
    void ToggleManualOverride();

    void ControlCompressorEnabledState();

    void DriveWithJoystick();
    void ControlIntakeMotor();
    void SetArmTargetAngle();
    void ControlIntakePiston();
    void ControlHatchPiston();
    void ControlArmMotor();
    void AutoControlArmMotor();
    void ManualControlArmMotor();

    void DisplayShuffleBoardInformation();
};