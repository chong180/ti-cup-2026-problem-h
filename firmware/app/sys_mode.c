/**
 * @file    AB.c
 * @brief   2024电赛H题第一问 — A到B陀螺仪角度环直行
 *
 * 状态机:
 *   IDLE --S1--> LEAVE --连续全白200ms--> RUN --宽限期后检测黑线--> DONE
 *    ^                                                              |
 *    +-----------------------------S1------------------------------+
 *
 * 角度PID环: error = targetYaw - currentYaw (归一化到-180~180)
 *            integral += error * dt
 *            derivative = (error - prevError) / dt
 *            correction = Kp*error + Ki*integral + Kd*derivative
 *            left  = baseSpeed + correction
 *            right = baseSpeed - correction
 *
 * 方向说明:
 *   yawError > 0 (车体偏左, Yaw比目标小) -> correction > 0
 *   -> 左轮加速 + 右轮减速 -> 车体右转 -> 纠正回目标航向
 *   yawError < 0 (车体偏右, Yaw比目标大) -> correction < 0
 *   -> 左轮减速 + 右轮加速 -> 车体左转 -> 纠正回目标航向
 *
 * @note 若车体仍然发散, 将AB_Update中left/right的+/-互换即可
 */
#include "app/sys_mode.h"
#include "legacy/angle_source.h"
#include "drivers/encoder_odometry.h"
#include "drivers/motor_tb6612.h"
#include "drivers/timebase.h"
#include "ball_balance/ball_hf_zero.h"

/* ============================================================
 *  模块内部状态
 * ============================================================ */
static SysMode   s_mode            = MODE_LINE_TRACK;
static AB_State  s_abState         = AB_IDLE;
static float     s_targetYaw       = 0.0f;
static uint8_t   s_abRunning       = 0U;
static uint32_t  s_abStartMs       = 0U;

/* 角度环PID状态 */
static float     s_yawIntegral     = 0.0f;   /* 积分项 */
static float     s_yawPrevError    = 0.0f;   /* 上次误差 (微分用) */

/* LEAVE->RUN 确认计数器: 连续全白的毫秒数 */
static uint16_t  s_leaveWhiteCount = 0U;

/* RUN 宽限期计数器: 进入RUN后倒计时, 期间忽略黑线 */
static uint16_t  s_runGraceCount   = 0U;

/* ============================================================
 *  内部工具函数
 * ============================================================ */

/* 角度归一化到 [-180, 180) */
static float wrapAngle180(float deg)
{
    while (deg >= 180.0f)  deg -= 360.0f;
    while (deg < -180.0f)  deg += 360.0f;
    return deg;
}

/* 速度限幅到 +/-TB6612_MAX_SPEED_PERCENT */
static int8_t clampSpeed(int16_t speed)
{
    if (speed >  AB_MAX_SPEED) return  AB_MAX_SPEED;
    if (speed < -AB_MAX_SPEED) return -AB_MAX_SPEED;
    return (int8_t)speed;
}

/* ============================================================
 *  API 实现
 * ============================================================ */

void AB_Init(void)
{
    s_mode            = MODE_LINE_TRACK;
    s_abState         = AB_IDLE;
    s_abRunning       = 0U;
    s_targetYaw       = 0.0f;
    s_leaveWhiteCount = 0U;
    s_runGraceCount   = 0U;
    s_yawIntegral     = 0.0f;
    s_yawPrevError    = 0.0f;
    s_abStartMs       = 0U;
}

void AB_SwitchMode(void)
{
    /* 循环顺序:
     *   TRACK -> HFZERO -> ZERO -> +5>-5 -> DRIVE1 -> HOLD6 -> 006
     *   -> AB -> AB20 -> ABCD -> TRACK50 -> REMOTE -> TRACK
     */
    TB6612_Stop();
    switch (s_mode) {
    case MODE_LINE_TRACK: s_mode = MODE_HF_ZERO; break;
    case MODE_HF_ZERO:    s_mode = MODE_BALL_LINE; break;
    case MODE_BALL_LINE:  s_mode = MODE_BALL_ROUTE; break;
    case MODE_BALL_ROUTE: s_mode = MODE_DRIVE1; break;
    case MODE_DRIVE1:     s_mode = MODE_HOLD6; break;
    case MODE_HOLD6:      s_mode = MODE_006; break;
    case MODE_006:        s_mode = MODE_007; break;
    case MODE_007:        s_mode = MODE_5NEW; break;
    case MODE_5NEW:       s_mode = MODE_006_1; break;
    case MODE_006_1:      s_mode = MODE_AB; break;
    case MODE_AB:         s_mode = MODE_BALL_AB20; break;
    case MODE_BALL_AB20:  s_mode = MODE_ABCD; break;
    case MODE_ABCD:       s_mode = MODE_TRACK50; break;
    case MODE_TRACK50:    s_mode = MODE_REMOTE; break;
    default:              s_mode = MODE_LINE_TRACK; break;
    }
    s_abState         = AB_IDLE;
    s_abRunning       = 0U;
    s_leaveWhiteCount = 0U;
    s_runGraceCount   = 0U;
}

SysMode  AB_GetMode(void)      { return s_mode; }

void AB_SetMode(SysMode m)
{
    /* 只接受现存模式; 已删除的编号(2/3/4)及其他非法值直接忽略 */
    switch (m) {
    case MODE_LINE_TRACK:
    case MODE_AB:
    case MODE_BALL_LINE:
    case MODE_BALL_ROUTE:
    case MODE_BALL_AB20:
    case MODE_TRACK50:
    case MODE_REMOTE:
    case MODE_DRIVE1:
    case MODE_ABCD:
    case MODE_HF_ZERO:
    case MODE_006:
    case MODE_007:
    case MODE_5NEW:
    case MODE_006_1:
        break;
    default:
        return;
    }

    TB6612_Stop();
    s_mode            = m;
    s_abState         = AB_IDLE;
    s_abRunning       = 0U;
    s_leaveWhiteCount = 0U;
    s_runGraceCount   = 0U;
}
AB_State AB_GetState(void)     { return s_abState; }
float    AB_GetTargetYaw(void) { return s_targetYaw; }

void AB_OnStartStop(void)
{
    if (s_mode != MODE_AB) {
        return;
    }

    if (s_abState == AB_IDLE || s_abState == AB_DONE) {
        /* 启动: 锁定当前航向角为直行目标 */
        s_targetYaw       = Angle_GetYaw();
        s_abState         = AB_RUN;
        s_abRunning       = 1U;
        s_leaveWhiteCount = 0U;
        s_runGraceCount   = 0U;
        s_yawIntegral     = 0.0f;
        s_yawPrevError    = 0.0f;
        s_abStartMs       = Timebase_GetMillis();
        EncoderDistance_Reset();
        HFZero_SetFeedforwardAngleDeg(0.0f);
        HFZero_SetEnabled(true);
    } else {
        /* 停止 */
        TB6612_Stop();
        s_abState         = AB_IDLE;
        s_abRunning       = 0U;
        s_leaveWhiteCount = 0U;
        s_runGraceCount   = 0U;
        s_abStartMs       = 0U;
        HFZero_SetFeedforwardAngleDeg(0.0f);
        HFZero_SetEnabled(false);
    }
}

void AB_Update(const LineTracker_State *line, int8_t *left, int8_t *right)
{
    uint32_t elapsedMs;
    int16_t baseSpeed;
    float distanceMm;
    float remainingMm;

    *left  = 0;
    *right = 0;

    if (s_mode != MODE_AB || s_abRunning == 0U) {
        return;
    }

    elapsedMs = Timebase_GetMillis() - s_abStartMs;
    if (elapsedMs < AB_BALL_SETTLE_MS) {
        TB6612_Stop();
        HFZero_SetFeedforwardAngleDeg(0.0f);
        return;
    }

    elapsedMs -= AB_BALL_SETTLE_MS;
    if (elapsedMs < AB_SOFT_START_MS) {
        baseSpeed = AB_START_SPEED +
            (int16_t)(((uint32_t)(AB_BASE_SPEED - AB_START_SPEED) *
                       elapsedMs) / AB_SOFT_START_MS);
        HFZero_SetFeedforwardAngleDeg(AB_ACCEL_BALL_FF_DEG);
    } else {
        baseSpeed = AB_BASE_SPEED;
        HFZero_SetFeedforwardAngleDeg(0.0f);
    }

    distanceMm = EncoderDistance_GetAverageMm();
    remainingMm = AB_TARGET_DISTANCE_MM - distanceMm;
    if ((remainingMm > 0.0f) &&
        (remainingMm < AB_SOFT_STOP_DISTANCE_MM)) {
        baseSpeed = AB_STOP_SPEED +
            (int16_t)(((float)(AB_BASE_SPEED - AB_STOP_SPEED) *
                       remainingMm) / AB_SOFT_STOP_DISTANCE_MM);
        HFZero_SetFeedforwardAngleDeg(AB_BRAKE_BALL_FF_DEG);
    }

    /* AB段按双轮平均编码器距离行驶1.5m后自动停车。 */
    if (EncoderDistance_GetAverageMm() >= AB_TARGET_DISTANCE_MM) {
        TB6612_Stop();
        HFZero_SetFeedforwardAngleDeg(0.0f);
        s_abState   = AB_DONE;
        s_abRunning = 0U;
        return;
    }

    /* ================================================================
     *  状态转移逻辑
     * ================================================================ */

    /* ---- LEAVE: 驶离A黑线 ----
     * 要求连续 AB_LEAVE_CONFIRM_MS 毫秒全白, 才确认已驶离A
     * 防止车体侧偏时传感器扫到A黑线导致误判 */
    if (s_abState == AB_LEAVE) {
        if (line->lineFound == 0U) {
            s_leaveWhiteCount++;
        } else {
            s_leaveWhiteCount = 0U;
        }

        if (s_leaveWhiteCount >= AB_LEAVE_CONFIRM_MS) {
            s_abState         = AB_RUN;
            s_runGraceCount   = AB_RUN_GRACE_MS;
            s_leaveWhiteCount = 0U;
        }
    }

    /* ---- RUN: 宽限期后检测B黑线 ----
     * 进入RUN后先等 AB_RUN_GRACE_MS 毫秒, 确保车已远离A
     * 宽限期过后, 任一传感器检测到黑线 -> 到达B, 停车 */
    /* ================================================================
     *  陀螺仪角度PID环直行 (LEAVE和RUN状态均执行)
     * ================================================================ */
    if (s_abState == AB_LEAVE || s_abState == AB_RUN) {
        float yawError = wrapAngle180(s_targetYaw - Angle_GetYaw());
        float correction;

        /* PID计算 (dt = 1ms = 0.001s) */
        s_yawIntegral += yawError * 0.001f;
        float derivative = (yawError - s_yawPrevError) / 0.001f;
        s_yawPrevError = yawError;

        if (line->lineFound != 0U) {
            /* 检测到线时按灰度误差循迹纠偏。 */
            correction = AB_LINE_KP * (float)line->error;
        } else {
            /* 短暂丢线时用锁定的陀螺仪航向继续低速直行。 */
            correction = AB_KP * yawError
                       + AB_KI * s_yawIntegral
                       + AB_KD * derivative;
        }

        /* 限幅 */
        if (correction >  AB_CORR_LIMIT) correction =  AB_CORR_LIMIT;
        if (correction < -AB_CORR_LIMIT) correction = -AB_CORR_LIMIT;

        /*
         * 角度环方向说明:
         *   yawError > 0 (车偏左) -> correction > 0
         *   -> left加速 + right减速 -> 右转纠正
         *   yawError < 0 (车偏右) -> correction < 0
         *   -> left减速 + right加速 -> 左转纠正
         */
        *left  = clampSpeed((int16_t)(baseSpeed + correction));
        *right = clampSpeed((int16_t)(baseSpeed - correction));
        TB6612_SetSpeed(*left, *right);
    }
}
