#ifndef __BALL_ZERO_CONTROL_H
#define __BALL_ZERO_CONTROL_H

#include <stdbool.h>

void ZeroControl_Init(void);
void ZeroControl_SetEnabled(bool enabled);
void ZeroControl_Toggle(void);
void ZeroControl_Update(void);
void ZeroControl_SetTargetMm(float targetMm);
void ZeroControl_SetFeedforwardAngleDeg(float angleDeg);
void ZeroControl_SetAngleSlewDegPerTick(float stepDeg);
void ZeroControl_AdjustPositionKp(float delta);

bool ZeroControl_IsEnabled(void);
float ZeroControl_GetPositionMm(void);
float ZeroControl_GetTargetMm(void);
float ZeroControl_GetFeedforwardAngleDeg(void);
float ZeroControl_GetVelocityMmS(void);
float ZeroControl_GetTargetVelocityMmS(void);
float ZeroControl_GetAngleDeg(void);
float ZeroControl_GetPositionKp(void);
float ZeroControl_GetPositionKd(void);
float ZeroControl_GetVelocityKp(void);
float ZeroControl_GetVelocityKd(void);

#endif
