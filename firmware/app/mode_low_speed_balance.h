/**
 * @file    mode_low_speed_balance.h
 * @brief   低速平衡循迹：沿用 Drive1 的循迹/定距结构，并将钢球稳定在零点。
 */
#ifndef __MODE_LOW_SPEED_BALANCE_H
#define __MODE_LOW_SPEED_BALANCE_H

#include "ti_msp_dl_config.h"
#include "drivers/line_tracker.h"

/* 任务参数：默认与 Drive1 一样行驶 450 cm。 */
#define DRIVE_BALANCE_TARGET_MM          (4500.0f)
#define DRIVE_BALANCE_MIN_BLACK_SENSORS  (3U)

/*
 * 速度显著低于 Drive1 的 30%。
 * 若实车静摩擦导致 18% 无法起步，可只提高此宏，不需要改控制逻辑。
 */
#define DRIVE_BALANCE_SPEED_PERCENT      (18)
#define DRIVE_BALANCE_MAX_SPEED          (28)

/* 低速循迹 PID 参数。 */
#define DRIVE_BALANCE_KP_DEFAULT         (0.35f)
#define DRIVE_BALANCE_KI_DEFAULT         (0.0f)
#define DRIVE_BALANCE_KD_DEFAULT         (0.0f)
#define DRIVE_BALANCE_KP_STEP            (0.05f)
#define DRIVE_BALANCE_CORRECTION_LIMIT   (10.0f)
#define DRIVE_BALANCE_LOST_CORRECTION    (10.0f)

/* 钢球目标为 0 mm，允许偏差为正负 10 mm（正负 1 cm）。 */
#define DRIVE_BALANCE_BALL_TARGET_MM     (0.0f)
#define DRIVE_BALANCE_BALL_TOLERANCE_MM  (10.0f)

typedef enum {
    DRIVE_BALANCE_IDLE = 0,
    DRIVE_BALANCE_RUN,
    DRIVE_BALANCE_DONE,
} DriveBalance_State;

void DriveBalance_Init(void);
void DriveBalance_OnStartStop(void);
void DriveBalance_Update(const LineTracker_State *line,
                         int8_t *left, int8_t *right);

uint8_t DriveBalance_IsRunning(void);
DriveBalance_State DriveBalance_GetState(void);
uint8_t DriveBalance_StartRejected(void);
uint8_t DriveBalance_GetBlackCount(void);
float DriveBalance_GetDistanceMm(void);
float DriveBalance_GetKp(void);
void DriveBalance_AdjustKp(float delta);

/* 仅当摄像头可见钢球且位置处于零点正负 1 cm 内时返回 1。 */
uint8_t DriveBalance_IsBallBalanced(void);
float DriveBalance_GetBallErrorMm(void);

#endif /* __DRIVE_BALANCE_H */
