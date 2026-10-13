#ifndef BALL_BALANCE_H
#define BALL_BALANCE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * MODE_BALL_LINE uses a timed open-loop 0 -> -5 cm -> +5 cm trajectory.
 * AB center control keeps using camera position feedback.
 */
void BallBalance_Init(void);
void BallBalance_Update(void);
void BallBalance_ResetSequence(void);
void BallBalance_StartCenterControl(void);
void BallBalance_SetCarFeedforwardDeg(float angleDeg);
void BallBalance_SetEnabled(bool enabled);
void BallBalance_SetTargetMm(float targetMm);
void BallBalance_SetPid(float kp, float ki, float kd);
float BallBalance_GetPositionMm(void);
float BallBalance_GetTargetMm(void);
float BallBalance_GetOutputRpm(void);
float BallBalance_GetMotorAngleDeg(void);
bool BallBalance_IsBallVisible(void);
bool BallBalance_IsFinalTarget(void);
bool BallBalance_IsEnabled(void);
float BallBalance_GetVisionPositionMm(void);
uint32_t BallBalance_GetVisionFrameSequence(void);
uint32_t BallBalance_GetVisionFrameMs(void);

#endif
