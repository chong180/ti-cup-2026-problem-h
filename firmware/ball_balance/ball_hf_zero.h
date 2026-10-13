#ifndef __BALL_HF_ZERO_H
#define __BALL_HF_ZERO_H

#include <stdbool.h>

typedef enum {
    HF_ZERO_IDLE = 0,
    HF_ZERO_ACQUIRE,
    HF_ZERO_HOLD,
    HF_ZERO_FAST,
    HF_ZERO_LOST
} HFZero_State;

void HFZero_Init(void);
void HFZero_SetEnabled(bool enabled);
void HFZero_Toggle(void);
void HFZero_Update(void);
void HFZero_SetFeedforwardAngleDeg(float angleDeg);
void HFZero_SetFeedforwardClampDeg(float clampDeg);
void HFZero_SetAngleLimitDeg(float limitDeg);
void HFZero_SetManualSlewDeg(float slewDeg);
void HFZero_SetStandaloneOscillationMode(bool enabled);
void HFZero_AdjustPositionGain(float delta);
void HFZero_SetTargetMm(float mm);
void HFZero_SetPositionGain(float gain);

bool HFZero_IsEnabled(void);
HFZero_State HFZero_GetState(void);
float HFZero_GetPositionMm(void);
float HFZero_GetVelocityMmS(void);
float HFZero_GetAngleDeg(void);
float HFZero_GetPositionGain(void);
float HFZero_GetFeedforwardAngleDeg(void);

#endif
