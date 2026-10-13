/**
 * @file    mode_fast_track.h
 * @brief   高速循迹模式 — 50% PWM最大速度
 *
 * 在基础循迹模式(MODE_LINE_TRACK)的基础上, 将电机PWM最大速度提升到50%,
 * PID参数相应调整以适应更高速度。
 *
 * 控制逻辑与基础循迹完全相同:
 *   error > 0 (线在右侧) -> correction > 0 -> left加速 + right减速 -> 右转
 *   error < 0 (线在左侧) -> correction < 0 -> left减速 + right加速 -> 左转
 *   丢线时按上次误差方向施加固定校正量回拐找线
 *
 * 按键:
 *   S1(PA25) 启动/停止
 *   S3(PB23) 增大Kp
 *   S4(PA29) 减小Kp
 */
#ifndef __MODE_FAST_TRACK_H
#define __MODE_FAST_TRACK_H

#include "ti_msp_dl_config.h"
#include "drivers/line_tracker.h"
#include "app/sys_mode.h"

/* ======================== 速度参数 ======================== */
#define TRACK50_SPEED_PERCENT     (50)      /* 基础循迹速度 (%) */
#define TRACK50_MAX_SPEED         (50)      /* 速度上限 (%)      */

/* ======================== PID参数 ======================== */
#define TRACK50_KP_DEFAULT        (1.4f)    /* P增益 (高速需要更大P) */
#define TRACK50_KI_DEFAULT        (0.0f)
#define TRACK50_KD_DEFAULT        (0.4f)
#define TRACK50_KP_STEP           (0.05f)   /* S3/S4调节步进 */
#define TRACK50_CORRECTION_LIMIT  (70.0f)   /* PID校正限幅 (与速度等比放大) */
#define TRACK50_LOST_CORRECTION   (50.0f)   /* 丢线固定校正量 */

/* ======================== API函数 ======================== */

void Track50_Init(void);
void Track50_OnStartStop(void);
uint8_t Track50_IsRunning(void);
float Track50_GetKp(void);
float Track50_GetKi(void);
float Track50_GetKd(void);
void Track50_AdjustKp(float delta);
void Track50_Update(const LineTracker_State *line, int8_t *left, int8_t *right);

#endif /* __TRACK50_H */
