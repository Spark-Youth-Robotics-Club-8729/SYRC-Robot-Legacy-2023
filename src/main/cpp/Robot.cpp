// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#if 0
#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
#endif

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <Robot.h>

void Robot::RobotInit()
{
    //invert right motors
    m_right.SetInverted(true); // if you want to invert motor outputs, you must do so here

#if 0
    // camera settings
    m_camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    m_camera.SetResolution(CAMERA_RES_W, CAMERA_RES_H);
    m_camera.SetFPS(CAMERA_FPS);
#endif

    // compressor settings
    m_compressor->SetClosedLoopControl(false);

    // intake motor settings
    m_intakeMotor.EnableDeadbandElimination(false);
    m_intakeMotor.SetSpeed(0);

    // intake hatch settings
    m_hatchMotor.EnableDeadbandElimination(false);
    m_hatchMotor.SetSpeed(0);

    //lift encoder settings
    // m_liftEncoder.Reset();
    // m_liftEncoder.SetMaxPeriod(1.0);
    // m_liftEncoder.SetMinRate(10);
    m_drivetrainEncoder1.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
    m_drivetrainEncoder2.SetDistancePerPulse((3.14159265358 * 6) / 360.0);
    //    wpi::outs() << "SetDistancePerPulse=" << m_liftEncoder.GetDistancePerPulse() << "\n";
    //    m_liftEncoder.SetReverseDirection(true);
    //    m_liftEncoder2.SetReverseDirection(true);


    // Encoder Electrical
    // 1 - Yellow
    // 0 - Blue

    //m_liftEncoder.SetSamplesToAverage(10);

    // encoder PID settings
    m_encoderPID.SetTargetAngle(0);
    m_encoderPID.SetTargetSpeed(ARM_SPEED);
}

void Robot::RobotPeriodic()
{
    /*
    ToggleManualOverride();
    SetArmTargetAngle();
    ControlArmMotor();
    ControlCompressorEnabledState();
    ControlIntakePiston();
    ControlIntakeMotor();
    ControlHatchMotor();
    DriveWithJoystick();
    DisplayShuffleBoardInformation();
    */
#if 0
    ControlHatchPiston();
//    m_compressor->SetClosedLoopControl(true);
#endif
}

void Robot::AutonomousInit() {
    wpi::outs() << "AutonomousInit() \n";

    m_drivetrainEncoder1.Reset();
    m_drivetrainEncoder2.Reset();
    arrivedDestination = false;
}

void Robot::AutonomousPeriodic() {
    wpi::outs() << "Encoder 1 Distance: " << wpi::format("%0.2f", m_drivetrainEncoder1.GetDistance()) << "\n";

    //drive forward 10 inches and reverse drive 10 inches
    if(m_drivetrainEncoder1.GetDistance() < 10 && arrivedDestination == false) {
        m_robotDrive.ArcadeDrive(0, 0.4);
    } else {
        //m_robotDrive.ArcadeDrive(0, 0);
        arrivedDestination = true;
        if(m_drivetrainEncoder1.GetDistance() > 0) {
            m_robotDrive.ArcadeDrive(0, -0.4);
        }
        else {
            m_robotDrive.ArcadeDrive(0, 0);
        }
    }
}

void Robot::TeleopInit() {
    wpi::outs() << "RobotInit() \n";

    m_drivetrainEncoder1.Reset();
    m_drivetrainEncoder2.Reset();
}

void Robot::TeleopPeriodic() {
    ToggleManualOverride();
    SetArmTargetAngle();
    ControlArmMotor();
    ControlCompressorEnabledState();
    ControlIntakePiston();
    ControlIntakeMotor();
    ControlHatchMotor();
    DriveWithJoystick();

    //normal drive; print out distance
    wpi::outs() << "Encoder 1 Distance: " << wpi::format("%0.2f", m_drivetrainEncoder1.GetDistance()) << "\n";
    wpi::outs() << "Encoder 2 Distance: " << wpi::format("%0.2f", m_drivetrainEncoder2.GetDistance()) << "\n";

    //DisplayShuffleBoardInformation();    
}

void Robot::TestPeriodic() {}

void Robot::ToggleManualOverride()
{
    manualOverride = m_xbox.GetRawAxis(MANUAL_OVERRIDE_TRIGGER) > 0.7;
    wpi::outs() << "manualOverride: " << m_xbox.GetRawAxis(MANUAL_OVERRIDE_TRIGGER) << "\n";
}

#if 1
void Robot::DriveWithJoystick()
{
    // acrade drive
    float xDrive = m_xbox.GetY() * DRIVE_X_SPEED * 0.7;
    float yDrive = m_xbox.GetX() * DRIVE_Y_SPEED * 0.9;
    m_robotDrive.ArcadeDrive(-yDrive, -xDrive); // hardware flipped; should be (yDrive, xDrive)
}
#endif

#if 1
void Robot::ControlIntakeMotor()
{
    /*
    if  (m_stick.GetRawButton(INTAKE_EX)) {
        m_intakeMotor.SetSpeed(0.4);
    }
    else if (m_stick.GetRawButton(INTAKE_RE)) {
        m_intakeMotor.SetSpeed(-0.4);
    }
    else if (m_stick.GetRawButton(INTAKE_OFF)) 
    {
        m_intakeMotor.SetSpeed(0.0);
    }
    */
}
#endif

#if 1
void Robot::ControlHatchMotor()
{
    /*
    if  (m_stick.GetRawButton(HATCH_FORW)) {
        m_hatchMotor.SetSpeed(0.2);
    }
    else if (m_stick.GetRawButton(HATCH_BACK)) {
        m_hatchMotor.SetSpeed(-0.2);
    }
    else if (m_stick.GetRawButton(HATCH_OFF)) {
        m_hatchMotor.SetSpeed(0.0);
    }
    */
}

#endif

void Robot::SetArmTargetAngle()
{
    bool height1 = m_xbox.GetRawButton(ARM_HEIGHT_1_BUTTON);
    bool height2 = m_xbox.GetRawButton(ARM_HEIGHT_2_BUTTON);
    bool height3 = m_xbox.GetRawButton(ARM_HEIGHT_3_BUTTON);
    bool height4 = m_xbox.GetRawButton(ARM_HEIGHT_4_BUTTON);

    if (height1) {
        m_encoderPID.SetTargetAngle(ARM_HEIGHT_1);
        return;
    }
    if (height2) {
        m_encoderPID.SetTargetAngle(ARM_HEIGHT_2);
        return;
    }
    if (height3) {
        m_encoderPID.SetTargetAngle(ARM_HEIGHT_3);
        return;
    }
    if (height4) {
        m_encoderPID.SetTargetAngle(ARM_HEIGHT_4);
        return;
    }
}

#if 1
 void Robot::ControlArmMotor()
{
    //m_encoderPID.SetEncoderValue(m_liftEncoder.Get());
    //m_encoderPID.SetEncoderValue(m_liftEncoder2.Get());
    if (manualOverride) {

    //    frc::SmartDashboard::PutNumber("ArmControl ", 100);
    //     ManualControlArmMotor();
        frc::SmartDashboard::PutNumber("ArmControl ", 1);
        AutoControlArmMotor();
    }
    else {
        frc::SmartDashboard::PutNumber("ArmControl ", 100);
        ManualControlArmMotor();
    //    frc::SmartDashboard::PutNumber("ArmControl ", 1);
    //    AutoControlArmMotor();
    }
    // m_encoderPID.SetEncoderValue(m_liftEncoder.Get());
    // if (manualOverride) {
    //     ManualControlArmMotor();
    // } else {
    //     AutoControlArmMotor();
    // }
}
#endif
void Robot::AutoControlArmMotor()
{
    float speed = m_encoderPID.GetSpeed();

    if (m_encoderPID.AtTargetAngle()) {

        m_armMotor.Set(ControlMode::PercentOutput, 0);
        m_armMotor2.Set(ControlMode::PercentOutput, 0);

    } else if (m_encoderPID.GetCurrentAngle() < m_encoderPID.GetTargetAngle()) {

        m_armMotor.Set(ControlMode::PercentOutput, ARM_SPEED * speed);
        m_armMotor2.Set(ControlMode::PercentOutput, -ARM_SPEED * speed);

    } else {

        m_armMotor.Set(ControlMode::PercentOutput, -ARM_SPEED * speed);
        m_armMotor2.Set(ControlMode::PercentOutput, ARM_SPEED * speed);
    }
}

void Robot::ManualControlArmMotor()
{
    frc::SmartDashboard::PutNumber("ArmControl ", 2);
    //m_armMotor.Set(ControlMode::PercentOutput, ARM_SPEED * m_stick.GetRawAxis(MANUAL_LIFT_AXIS) + 0.2);
    //m_armMotor2.Set(ControlMode::PercentOutput, -ARM_SPEED * m_stick.GetRawAxis(MANUAL_LIFT_AXIS) - 0.2);
    //wpi::outs() << "Encoder Reset \n";
    //m_liftEncoder.Reset();
    //m_liftEncoder2.Reset();
    m_encoderPID.SetTargetAngle(0);
}

#if 1
void Robot::ControlCompressorEnabledState()
{
    /*
    // turn on compressor
    if (m_stick.GetRawButton(COMPRESSOR_ON_BUTTON)) {
        m_compressor->SetClosedLoopControl(true);
    }
    // turn off compressor
    else if (m_stick.GetRawButton(COMPRESSOR_OFF_BUTTON)) {
        m_compressor->SetClosedLoopControl(false);
    //  m_compressor->Stop();
    }
    */
}
#endif

void Robot::ControlIntakePiston()
{
    /*
    frc::SmartDashboard::PutBoolean("intake ", 120);
//   m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);

    // extend piston
    if (m_stick.GetRawButton(SOLENOID_ON)) {
    //(m_xbox.GetPOV(0) == 315 || m_xbox.GetPOV(0) == 45 || m_xbox.GetPOV(0) == 0) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    // retract piston
    else if (m_stick.GetRawButton(SOLENOID_OFF)) {
    //(m_xbox.GetPOV(0) >= 135 && m_xbox.GetPOV(0) <= 225) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    // do nothing with piston
    else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
    */
}

void Robot::DisplayShuffleBoardInformation()
{ 
    // newdistance = std::to_string(distance);
    // frc::SmartDashboard::PutString("Encoder Distance Way 2", newdistance);
    // distance = m_liftEncoder.GetDistance() + distance; 

    frc::SmartDashboard::PutNumber("Encoder Distance", distance);
    frc::SmartDashboard::PutNumber("Encoder2 Distance", distance2);
    //distance = m_liftEncoder.GetDistance() + distance;
    //distance2 = m_liftEncoder2.GetDistance() + distance2;

}


#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif

#if 0
#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.wr
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
#endif
