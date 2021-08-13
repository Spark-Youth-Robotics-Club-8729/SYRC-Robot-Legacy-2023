#pragma once

#include <frc/WPILib.h>

class CustomPID {

private:
    const float ARM_COUNTS_PER_DEGREE = 7 * 71 / 360.0;
    const float ERROR_SLOWDOWN_MAX_ANGLE = 60;
    const float BUFFER_ANGLE = 0;

    float targetAngle = 0;
    float encoderValue = 0;
    float targetSpeed = 1;

public:
    float SetEncoderValue(float val);

    float GetCurrentAngle();
    float AngleOffset();

    float GetTargetAngle();
    void SetTargetAngle(float target);
    bool AtTargetAngle();

    void SetTargetSpeed(float speed);
    float GetSpeed();
};