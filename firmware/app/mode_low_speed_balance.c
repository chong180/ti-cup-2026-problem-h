/**
 * @file    drive_balance.c
 * @brief   Drive1 风格的低速循迹 + 编码器定距 + 钢球零点平衡。
 *
 * 本模块保持独立，不占用 AB.h 中的模式编号。主程序进入新模式时调用
 * DriveBalance_OnStartStop()，循环中调用 DriveBalance_Update()；
 * BallBalance_Update() 仍由主循环按原有方式持续调用。
 */
#include "app/mode_low_speed_balance.h"

#include "ball_balance/ball_balance.h"
#include "drivers/encoder_odometry.h"
#include "drivers/motor_tb6612.h"
#include "drivers/pid.h"

static uint8_t          s_running       = 0U;
static DriveBalance_State s_state       = DRIVE_BALANCE_IDLE;
static uint8_t          s_startRejected = 0U;
static uint8_t          s_blackCount    = 0U;
static int8_t           s_lastLineError = 0;
static PID_Controller   s_pid;

static int8_t clampForwardSpeed(int16_t speed)
{
    if (speed > DRIVE_BALANCE_MAX_SPEED) {
        return DRIVE_BALANCE_MAX_SPEED;
    }
    if (speed < 0) {
        return 0;
    }
    return (int8_t)speed;
}

static uint8_t countBlackSensors(void)
{
    LineTracker_State line;
    uint8_t mask;
    uint8_t count = 0U;

    LineTracker_Read(&line);
    mask = line.activeMask;
    while (mask != 0U) {
        count += (uint8_t)(mask & 1U);
        mask = (uint8_t)(mask >> 1);
    }
    return count;
}

static float absoluteFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void stopRun(DriveBalance_State nextState)
{
    TB6612_Stop();
    BallBalance_SetCarFeedforwardDeg(0.0f);
    BallBalance_SetEnabled(false);
    s_running = 0U;
    s_state = nextState;
    s_lastLineError = 0;
    PID_Reset(&s_pid);
}

void DriveBalance_Init(void)
{
    s_running = 0U;
    s_state = DRIVE_BALANCE_IDLE;
    s_startRejected = 0U;
    s_blackCount = 0U;
    s_lastLineError = 0;
    PID_Init(&s_pid,
             DRIVE_BALANCE_KP_DEFAULT,
             DRIVE_BALANCE_KI_DEFAULT,
             DRIVE_BALANCE_KD_DEFAULT,
             DRIVE_BALANCE_CORRECTION_LIMIT);
}

void DriveBalance_OnStartStop(void)
{
    if (s_running != 0U) {
        stopRun(DRIVE_BALANCE_IDLE);
        return;
    }

    s_blackCount = countBlackSensors();
    if (s_blackCount < DRIVE_BALANCE_MIN_BLACK_SENSORS) {
        s_startRejected = 1U;
        return;
    }

    EncoderDistance_Reset();
    PID_Reset(&s_pid);
    s_lastLineError = 0;
    s_startRejected = 0U;
    s_running = 1U;
    s_state = DRIVE_BALANCE_RUN;

    /*
     * 使用摄像头矩形框真实中心作为零点，目标固定为 0 mm。
     * 该接口会立即启用钢球闭环，不经过 +5 cm/-5 cm 序列。
     */
    BallBalance_SetCarFeedforwardDeg(0.0f);
    BallBalance_StartCenterControl();
}

void DriveBalance_Update(const LineTracker_State *line,
                         int8_t *left, int8_t *right)
{
    float correction;

    *left = 0;
    *right = 0;

    if (s_running == 0U) {
        return;
    }

    if (EncoderDistance_GetAverageMm() >= DRIVE_BALANCE_TARGET_MM) {
        stopRun(DRIVE_BALANCE_DONE);
        return;
    }

    if (line->lineFound != 0U) {
        s_lastLineError = line->error;
        correction = PID_Update(&s_pid, (float)line->error, 0.001f);
        *left = clampForwardSpeed(
            (int16_t)DRIVE_BALANCE_SPEED_PERCENT + (int16_t)correction);
        *right = clampForwardSpeed(
            (int16_t)DRIVE_BALANCE_SPEED_PERCENT - (int16_t)correction);
    } else if (s_lastLineError == 0) {
        TB6612_Stop();
        return;
    } else {
        PID_Reset(&s_pid);
        correction = (s_lastLineError < 0) ?
            -DRIVE_BALANCE_LOST_CORRECTION :
             DRIVE_BALANCE_LOST_CORRECTION;
        *left = clampForwardSpeed(
            (int16_t)DRIVE_BALANCE_SPEED_PERCENT + (int16_t)correction);
        *right = clampForwardSpeed(
            (int16_t)DRIVE_BALANCE_SPEED_PERCENT - (int16_t)correction);
    }

    TB6612_SetSpeed(*left, *right);
}

uint8_t DriveBalance_IsRunning(void)
{
    return s_running;
}

DriveBalance_State DriveBalance_GetState(void)
{
    return s_state;
}

uint8_t DriveBalance_StartRejected(void)
{
    return s_startRejected;
}

uint8_t DriveBalance_GetBlackCount(void)
{
    return s_blackCount;
}

float DriveBalance_GetDistanceMm(void)
{
    return EncoderDistance_GetAverageMm();
}

float DriveBalance_GetKp(void)
{
    return s_pid.kp;
}

void DriveBalance_AdjustKp(float delta)
{
    PID_AdjustGain(&s_pid, PID_GAIN_KP, delta);
}

float DriveBalance_GetBallErrorMm(void)
{
    return BallBalance_GetPositionMm() - DRIVE_BALANCE_BALL_TARGET_MM;
}

uint8_t DriveBalance_IsBallBalanced(void)
{
    if (!BallBalance_IsBallVisible()) {
        return 0U;
    }
    return (absoluteFloat(DriveBalance_GetBallErrorMm()) <=
            DRIVE_BALANCE_BALL_TOLERANCE_MM) ? 1U : 0U;
}
