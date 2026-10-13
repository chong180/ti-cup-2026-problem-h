/**
 * @file    drive1.c
 * @brief   驱动1模式 — 灰度循迹 + 编码器定距(440cm)停车
 *
 * 状态机:
 *   IDLE --S1且>=3路黑线--> RUN --平均里程>=4400mm--> DONE
 *    ^                       |
 *    +---------S1(停止)------+
 *
 * 循迹PID逻辑与基础循迹/Track50一致:
 *   error > 0 (线在右侧) -> correction > 0 -> left加速 + right减速 -> 右转
 *   error < 0 (线在左侧) -> correction < 0 -> left减速 + right加速 -> 左转
 *   丢线时按上次误差方向施加固定校正量回拐找线
 *
 * 里程来源: encoder_distance 模块 (A/B相上升沿2倍频, 无方向),
 *   启动时 EncoderDistance_Reset() 清零, 运行中每1ms检查
 *   EncoderDistance_GetAverageMm() 是否到达目标。
 */
#include "app/mode_fixed_distance.h"
#include "drivers/motor_tb6612.h"
#include "drivers/pid.h"
#include "drivers/encoder_odometry.h"
#include "app/sys_mode.h"

/* ============================================================
 *  模块内部状态
 * ============================================================ */
static uint8_t         s_running       = 0U;
static Drive1_State    s_state         = DRIVE1_IDLE;
static uint8_t         s_startRejected = 0U;  /* 上次S1因黑线不足被拒绝 */
static uint8_t         s_blackCount    = 0U;  /* 最近一次检测到的黑线路数 */
static int8_t          s_lastLineError = 0;
static PID_Controller  s_pid;

/* 速度限幅 */
static int8_t clampForwardSpeed(int16_t speed)
{
    if (speed >  DRIVE1_MAX_SPEED) return  DRIVE1_MAX_SPEED;
    if (speed < 0) return 0;
    return (int8_t)speed;
}

/* 统计8路灰度中检测到黑色的路数 */
static uint8_t countBlackSensors(void)
{
    LineTracker_State st;
    uint8_t mask;
    uint8_t count = 0U;

    LineTracker_Read(&st);          /* 纯GPIO读取, 按键上下文安全调用 */
    mask = st.activeMask;
    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask = (uint8_t)(mask >> 1);
    }
    return count;
}

/* ============================================================
 *  API 实现
 * ============================================================ */

void Drive1_Init(void)
{
    s_running       = 0U;
    s_state         = DRIVE1_IDLE;
    s_startRejected = 0U;
    s_blackCount    = 0U;
    s_lastLineError = 0;
    PID_Init(&s_pid, DRIVE1_KP_DEFAULT, DRIVE1_KI_DEFAULT, DRIVE1_KD_DEFAULT,
             DRIVE1_CORRECTION_LIMIT);
}

void Drive1_OnStartStop(void)
{
    if (AB_GetMode() != MODE_DRIVE1) {
        return;
    }

    if (s_running != 0U) {
        /* 运行中 -> 停止 */
        TB6612_Stop();
        s_running       = 0U;
        s_state         = DRIVE1_IDLE;
        s_lastLineError = 0;
        PID_Reset(&s_pid);
        return;
    }

    /* IDLE/DONE -> 尝试启动: 检查启动条件 (>=3路传感器见黑) */
    s_blackCount = countBlackSensors();
    if (s_blackCount < DRIVE1_MIN_BLACK_SENSORS) {
        s_startRejected = 1U;       /* 条件不满足, 拒绝启动 */
        return;
    }

    EncoderDistance_Reset();        /* 里程清零, 从本次启动开始累计 */
    PID_Reset(&s_pid);
    s_lastLineError = 0;
    s_startRejected = 0U;
    s_running       = 1U;
    s_state         = DRIVE1_RUN;
}

void Drive1_Update(const LineTracker_State *line, int8_t *left, int8_t *right)
{
    *left  = 0;
    *right = 0;

    if (AB_GetMode() != MODE_DRIVE1 || s_running == 0U) {
        return;
    }

    /* ---- 定距检查: 平均累计里程到达440cm -> 停车 ---- */
    if (EncoderDistance_GetAverageMm() >= DRIVE1_TARGET_MM) {
        TB6612_Stop();
        s_running = 0U;
        s_state   = DRIVE1_DONE;
        return;
    }

    /* ---- 灰度循迹 (与Track50相同的PID结构) ---- */
    if (line->lineFound != 0U) {
        /* 有线: PID跟随 */
        s_lastLineError = line->error;
        float correction = PID_Update(&s_pid, (float)line->error, 0.001f);
        *left  = clampForwardSpeed((int16_t)DRIVE1_SPEED_PERCENT + (int16_t)correction);
        *right = clampForwardSpeed((int16_t)DRIVE1_SPEED_PERCENT - (int16_t)correction);
    } else if (s_lastLineError == 0) {
        /* 丢线且无历史: 停车 */
        TB6612_Stop();
        return;
    } else {
        /* 丢线: 按上次误差方向固定校正找线 */
        PID_Reset(&s_pid);
        float correction = (s_lastLineError < 0) ? -DRIVE1_LOST_CORRECTION :
                            DRIVE1_LOST_CORRECTION;
        *left  = clampForwardSpeed((int16_t)DRIVE1_SPEED_PERCENT + (int16_t)correction);
        *right = clampForwardSpeed((int16_t)DRIVE1_SPEED_PERCENT - (int16_t)correction);
    }

    TB6612_SetSpeed(*left, *right);
}

uint8_t Drive1_IsRunning(void)      { return s_running; }
Drive1_State Drive1_GetState(void)  { return s_state; }
uint8_t Drive1_StartRejected(void)  { return s_startRejected; }
uint8_t Drive1_GetBlackCount(void)  { return s_blackCount; }

float Drive1_GetDistanceMm(void)
{
    return EncoderDistance_GetAverageMm();
}

float Drive1_GetKp(void)            { return s_pid.kp; }

void Drive1_AdjustKp(float delta)
{
    PID_AdjustGain(&s_pid, PID_GAIN_KP, delta);
}
