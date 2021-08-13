#include "CustomPID.h"

float CustomPID::SetEncoderValue(float val)
{
    encoderValue = val;
    return encoderValue;
}

float CustomPID::GetCurrentAngle()
{
    return encoderValue / ARM_COUNTS_PER_DEGREE;
}

float CustomPID::AngleOffset()
{
    return GetCurrentAngle() - GetTargetAngle();
}

float CustomPID::GetTargetAngle()
{
    return targetAngle;
}

void CustomPID::SetTargetAngle(float target)
{
    targetAngle = target;
}

bool CustomPID::AtTargetAngle()
{
    return abs(AngleOffset()) < BUFFER_ANGLE;
}

void CustomPID::SetTargetSpeed(float speed)
{
    targetSpeed = speed;
}

float CustomPID::GetSpeed()
{
    if (abs(AngleOffset()) < ERROR_SLOWDOWN_MAX_ANGLE) {

        return targetSpeed * (abs(AngleOffset()) - BUFFER_ANGLE) / ERROR_SLOWDOWN_MAX_ANGLE;
    }

    return targetSpeed;
}