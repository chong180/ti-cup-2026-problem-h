/**
 * @file    Track50.c
 * @brief   高速循迹模式 — 50% PWM最大速度
 *
 * 逻辑与基础循迹模式(empty.c中的updateMotor)完全相同,
 * 区别仅在于速度参数提升到50%, PID参数相应放大。
 *
 * 控制逻辑:
 *   error > 0 (线在右侧) -> correction > 0 -> left加速 + right减速 -> 右转
 *   error < 0 (线在左侧) -> correction < 0 -> left减速 + right加速 -> 左转
 *   丢线时按上次误差方向施加固定校正量回拐找线
 */
#include "app/mode_fast_track.h"
#include "drivers/motor_tb6612.h"
#include "drivers/pid.h"

/* ============================================================
 *  模块内部状态
 * ============================================================ */
static uint8_t   gRunning       = 0U;
static int8_t    gLastLineError = 0;
static PID_Controller gPid;

/* 速度限幅 */
static int8_t clampSpeed(int16_t speed)
{
    if (speed >  TRACK50_MAX_SPEED) return  TRACK50_MAX_SPEED;
    if (speed < -TRACK50_MAX_SPEED) return -TRACK50_MAX_SPEED;
    return (int8_t)speed;
}

/* ============================================================
 *  API 实现
 * ============================================================ */

void Track50_Init(void)
{
    gRunning       = 0U;
    gLastLineError = 0;
    PID_Init(&gPid, TRACK50_KP_DEFAULT, TRACK50_KI_DEFAULT, TRACK50_KD_DEFAULT,
             TRACK50_CORRECTION_LIMIT);
}

void Track50_OnStartStop(void)
{
    if (AB_GetMode() != MODE_TRACK50) {
        return;
    }

    gRunning = (uint8_t)(gRunning == 0U);
    if (gRunning == 0U) {
        TB6612_Stop();
        gLastLineError = 0;
        PID_Reset(&gPid);
    }
}

uint8_t Track50_IsRunning(void)  { return gRunning; }
float   Track50_GetKp(void)     { return gPid.kp; }
float   Track50_GetKi(void)     { return gPid.ki; }
float   Track50_GetKd(void)     { return gPid.kd; }

void Track50_AdjustKp(float delta)
{
    PID_AdjustGain(&gPid, PID_GAIN_KP, delta);
}

void Track50_Update(const LineTracker_State *line, int8_t *left, int8_t *right)
{
    *left  = 0;
    *right = 0;

    if (AB_GetMode() != MODE_TRACK50 || gRunning == 0U) {
        TB6612_Stop();
        return;
    }

    if (line->lineFound != 0U) {
        /* 有线: PID跟随 */
        gLastLineError = line->error;
        float correction = PID_Update(&gPid, (float)line->error, 0.001f);
        *left  = clampSpeed((int16_t)TRACK50_SPEED_PERCENT + (int16_t)correction);
        *right = clampSpeed((int16_t)TRACK50_SPEED_PERCENT - (int16_t)correction);
    } else if (gLastLineError == 0) {
        /* 丢线且无历史: 停车 */
        TB6612_Stop();
        return;
    } else {
        /* 丢线: 按上次误差方向固定校正找线 */
        PID_Reset(&gPid);
        float correction = (gLastLineError < 0) ? -TRACK50_LOST_CORRECTION :
                            TRACK50_LOST_CORRECTION;
        *left  = clampSpeed((int16_t)TRACK50_SPEED_PERCENT + (int16_t)correction);
        *right = clampSpeed((int16_t)TRACK50_SPEED_PERCENT - (int16_t)correction);
    }

    TB6612_SetSpeed(*left, *right);
}
