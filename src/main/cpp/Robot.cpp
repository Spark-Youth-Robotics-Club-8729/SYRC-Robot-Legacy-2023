/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::RobotInit()
{
    // camera settings
    m_camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    m_camera.SetResolution(CAMERA_RES_W, CAMERA_RES_H);
    m_camera.SetFPS(CAMERA_FPS);

    // intake motor settings
    m_intakeMotor.EnableDeadbandElimination(false);

    // compressor settings
    m_compressor->SetClosedLoopControl(true);

    //lift encoder settings
    m_liftEncoder.Reset();
    m_liftEncoder.SetMaxPeriod(0.1);
    m_liftEncoder.SetMinRate(10);
    m_liftEncoder.SetDistancePerPulse(5);
    m_liftEncoder.SetReverseDirection(false);
    m_liftEncoder.SetSamplesToAverage(10);

    // encoder PID settings
    m_encoderPID.SetTargetAngle(0);
    m_encoderPID.SetTargetSpeed(ARM_SPEED);
}

void Robot::RobotPeriodic()
{
    ToggleManualOverride();

    DriveWithJoystick();
    ControlIntakeMotor();
    SetArmTargetAngle();
    ControlArmMotor();

    //ControlCompressorEnabledState();
    ControlIntakePiston();
    ControlHatchPiston();

    DisplayShuffleBoardInformation();

    m_compressor->SetClosedLoopControl(true);
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::ToggleManualOverride()
{
    manualOverride = m_xbox.GetRawAxis(MANUAL_OVERRIDE_TRIGGER) > 0.7;
}

void Robot::DriveWithJoystick()
{
    // acrade drive
    float xDrive = m_stick.GetX() * DRIVE_X_SPEED * 0.7;
    float yDrive = m_stick.GetY() * DRIVE_Y_SPEED * 0.9;
    m_robotDrive.ArcadeDrive(xDrive, yDrive); // hardware flipped; should be (yDrive, xDrive)
}

void Robot::ControlIntakeMotor()
{
    if (manualOverride) {
        m_intakeMotor.SetSpeed(m_xbox.GetRawAxis(INTAKE_AXIS) * INTAKE_SPEED);
    }
    else if (!m_cargoSwitch.Get()) { // switch presssed
        m_intakeMotor.SetSpeed(std::fmax(m_xbox.GetRawAxis(INTAKE_AXIS) * INTAKE_SPEED, 0));
    }
    else {
        m_intakeMotor.SetSpeed(m_xbox.GetRawAxis(INTAKE_AXIS) * INTAKE_SPEED);
    }
}

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

void Robot::ControlArmMotor()
{
    m_encoderPID.SetEncoderValue(m_liftEncoder.Get());
    if (manualOverride) {
        ManualControlArmMotor();
    } else {
        AutoControlArmMotor();
    }
}

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
    m_armMotor.Set(ControlMode::PercentOutput, ARM_SPEED * m_xbox.GetRawAxis(MANUAL_LIFT_AXIS) + 0.2);
    m_armMotor2.Set(ControlMode::PercentOutput, -ARM_SPEED * m_xbox.GetRawAxis(MANUAL_LIFT_AXIS) - 0.2);
    m_liftEncoder.Reset();
    m_encoderPID.SetTargetAngle(0);
}

void Robot::ControlCompressorEnabledState()
{
    // turn on compressor
    if (m_stick.GetRawButton(COMPRESSOR_ON_BUTTON)) {
        m_compressor->SetClosedLoopControl(true);
    }
    // turn off compressor
    else if (m_stick.GetRawButton(COMPRESSOR_OFF_BUTTON)) {
        m_compressor->SetClosedLoopControl(false);
    }
}

void Robot::ControlIntakePiston()
{
    // extend piston
    if (m_xbox.GetPOV(0) == 315 || m_xbox.GetPOV(0) == 45 || m_xbox.GetPOV(0) == 0) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    // retract piston
    else if (m_xbox.GetPOV(0) >= 135 && m_xbox.GetPOV(0) <= 225) {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    // do nothing with piston
    else {
        m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
}

void Robot::ControlHatchPiston()
{
    if (m_xbox.GetPOV(0) >= 15 && m_xbox.GetPOV(0) <= 135) {
        m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    // retract piston
    else if (m_xbox.GetPOV(0) >= 225 && m_xbox.GetPOV(0) <= 315) {
        m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    // do nothing with piston
    else {
        m_hatchSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
    }
}

void Robot::DisplayShuffleBoardInformation()
{
    frc::SmartDashboard::PutBoolean("Compressor Enabled?", m_compressor->Enabled());
    frc::SmartDashboard::PutNumber("Arm Angle", m_encoderPID.GetCurrentAngle());
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif