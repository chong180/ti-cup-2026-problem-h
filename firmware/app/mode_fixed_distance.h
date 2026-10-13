/**
 * @file mode_fixed_distance.h
 * @brief 独立巡线模式：灰度循迹，编码器测距，450 cm 自动停车。
 */
#ifndef __MODE_FIXED_DISTANCE_H
#define __MODE_FIXED_DISTANCE_H

#include "ti_msp_dl_config.h"
#include "drivers/line_tracker.h"

/* 任务参数 */
#define DRIVE1_TARGET_MM          (4500.0f)
#define DRIVE1_MIN_BLACK_SENSORS  (3U)

/* 速度与PID参数 */
#define DRIVE1_SPEED_PERCENT      (30)
#define DRIVE1_MAX_SPEED          (50)
#define DRIVE1_KP_DEFAULT         (0.45f)
#define DRIVE1_KI_DEFAULT         (0.0f)
#define DRIVE1_KD_DEFAULT         (0.0f)   /* 避免离散灰度误差造成D项突跳 */
#define DRIVE1_KP_STEP            (0.05f)
#define DRIVE1_CORRECTION_LIMIT   (25.0f)  /* 小于30%基速，禁止车轮反转 */
#define DRIVE1_LOST_CORRECTION    (25.0f)

typedef enum {
    DRIVE1_IDLE = 0,
    DRIVE1_RUN,
    DRIVE1_DONE,
} Drive1_State;

void Drive1_Init(void);
void Drive1_OnStartStop(void);
void Drive1_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

uint8_t Drive1_IsRunning(void);
Drive1_State Drive1_GetState(void);
uint8_t Drive1_StartRejected(void);
uint8_t Drive1_GetBlackCount(void);
float Drive1_GetDistanceMm(void);
float Drive1_GetKp(void);
void Drive1_AdjustKp(float delta);

#endif /* __DRIVE1_H */
