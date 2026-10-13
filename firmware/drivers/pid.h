#ifndef __PID_H
#define __PID_H

typedef enum {
    PID_GAIN_KP = 0,
    PID_GAIN_KI,
    PID_GAIN_KD,
} PID_Gain;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previousError;
    float outputLimit;
} PID_Controller;

void PID_Init(PID_Controller *controller, float kp, float ki, float kd,
              float outputLimit);
void PID_Reset(PID_Controller *controller);
float PID_Update(PID_Controller *controller, float error, float dtSeconds);
float PID_UpdateWithDerivative(PID_Controller *controller, float error,
                               float errorDerivative, float dtSeconds);
void PID_AdjustGain(PID_Controller *controller, PID_Gain gain, float delta);

#endif
