#include "drivers/pid.h"

static float clamp(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

void PID_Init(PID_Controller *controller, float kp, float ki, float kd,
              float outputLimit)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->outputLimit = outputLimit;
    PID_Reset(controller);
}

void PID_Reset(PID_Controller *controller)
{
    controller->integral = 0.0f;
    controller->previousError = 0.0f;
}

float PID_Update(PID_Controller *controller, float error, float dtSeconds)
{
    float derivative;

    if (dtSeconds <= 0.0f) {
        return 0.0f;
    }

    derivative = (error - controller->previousError) / dtSeconds;
    return PID_UpdateWithDerivative(controller, error, derivative, dtSeconds);
}

float PID_UpdateWithDerivative(PID_Controller *controller, float error,
                               float errorDerivative, float dtSeconds)
{
    float output;

    if (dtSeconds <= 0.0f) {
        return 0.0f;
    }

    controller->integral += error * dtSeconds;
    if (controller->ki > 0.0f) {
        float integralLimit = controller->outputLimit / (2.0f * controller->ki);
        controller->integral = clamp(controller->integral, -integralLimit, integralLimit);
    }

    output = controller->kp * error + controller->ki * controller->integral +
             controller->kd * errorDerivative;
    controller->previousError = error;
    return clamp(output, -controller->outputLimit, controller->outputLimit);
}

void PID_AdjustGain(PID_Controller *controller, PID_Gain gain, float delta)
{
    switch (gain) {
    case PID_GAIN_KP:
        controller->kp = clamp(controller->kp + delta, 0.0f, 3.0f);
        break;
    case PID_GAIN_KI:
        controller->ki = clamp(controller->ki + delta, 0.0f, 2.0f);
        break;
    case PID_GAIN_KD:
        controller->kd = clamp(controller->kd + delta, 0.0f, 0.02f);
        break;
    default:
        break;
    }
    PID_Reset(controller);
}
