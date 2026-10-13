#include "ball_balance/ball_route_ramp.h"

#include <stdint.h>
#include "ball_balance/ball_zero_control.h"           /* ZeroControl API */
#include "ball_balance/ball_balance.h"
#include "drivers/timebase.h"    /* Timebase_GetMillis */

/* ======================== Parameters ======================== */

/* 目标位置 (mm): 0 → +50(+5cm) → -50(-5cm) */
#define ROUTE5_NEW_POSITIVE_MM        (50.0f)
#define ROUTE5_NEW_NEGATIVE_MM        (-50.0f)
#define ROUTE5_NEW_ZERO_MM            (0.0f)

/* 线性扫描时间 */
#define ROUTE5_NEW_RAMP_UP_MS         (500U)    /* 0→+50mm: 0.5s, 100mm/s */
#define ROUTE5_NEW_RAMP_DOWN_MS       (1000U)   /* +50→-50mm: 1.0s, 100mm/s */

/* 球到达终点后的稳定判定 */
#define ROUTE5_NEW_FINAL_TOL_MM       (3.0f)
#define ROUTE5_NEW_FINAL_SPEED_MMPS   (8.0f)
#define ROUTE5_NEW_FINAL_FRAMES       (15U)

/* 启动前馈角: 破静摩擦, 负角度驱动球向+方向 */
#define ROUTE5_NEW_START_FF_DEG       (-2.0f)

/* ======================== State ======================== */
static Route5New_State s_state;
static uint32_t s_phaseStartMs;
static float s_targetMm;
static uint32_t s_lastFrameSeq;
static uint8_t s_stableFrames;

static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

void Route5New_Init(void)
{
    s_state = ROUTE5_NEW_IDLE;
    s_phaseStartMs = 0U;
    s_targetMm = ROUTE5_NEW_ZERO_MM;
    s_lastFrameSeq = BallBalance_GetVisionFrameSequence();
    s_stableFrames = 0U;
}

void Route5New_Start(void)
{
    /* 初始目标=0, 加前馈破静摩擦 */
    ZeroControl_SetTargetMm(ROUTE5_NEW_ZERO_MM);
    ZeroControl_SetFeedforwardAngleDeg(ROUTE5_NEW_START_FF_DEG);
    ZeroControl_SetEnabled(true);
    s_targetMm = ROUTE5_NEW_ZERO_MM;
    s_phaseStartMs = Timebase_GetMillis();
    s_state = ROUTE5_NEW_RAMP_UP;
    s_stableFrames = 0U;
    s_lastFrameSeq = BallBalance_GetVisionFrameSequence();
}

void Route5New_Stop(void)
{
    ZeroControl_SetEnabled(false);
    ZeroControl_SetFeedforwardAngleDeg(0.0f);
    ZeroControl_SetTargetMm(ROUTE5_NEW_ZERO_MM);
    s_state = ROUTE5_NEW_IDLE;
    s_targetMm = ROUTE5_NEW_ZERO_MM;
    s_stableFrames = 0U;
}

void Route5New_Toggle(void)
{
    if ((s_state == ROUTE5_NEW_IDLE) || (s_state == ROUTE5_NEW_DONE)) {
        Route5New_Start();
    } else {
        Route5New_Stop();
    }
}

void Route5New_Update(void)
{
    uint32_t elapsedMs;
    uint32_t sequence;

    if ((s_state == ROUTE5_NEW_IDLE) || (s_state == ROUTE5_NEW_DONE)) return;

    /* 球不可见时清除稳定计数, 但不停止扫描 */
    if (!BallBalance_IsBallVisible()) {
        s_stableFrames = 0U;
        /* 丢球时仍继续线性扫描目标, 控制器会保持上次角度 */
        return;
    }

    /* 球可见后清除启动前馈 */
    if (ZeroControl_GetFeedforwardAngleDeg() != 0.0f) {
        ZeroControl_SetFeedforwardAngleDeg(0.0f);
    }

    /* 只在新相机帧处理 */
    sequence = BallBalance_GetVisionFrameSequence();
    if (sequence == s_lastFrameSeq) return;
    s_lastFrameSeq = sequence;

    elapsedMs = Timebase_GetMillis() - s_phaseStartMs;

    switch (s_state) {

    case ROUTE5_NEW_RAMP_UP:
        /* 线性: 0 → +50mm over RAMP_UP_MS */
        if (elapsedMs >= ROUTE5_NEW_RAMP_UP_MS) {
            s_targetMm = ROUTE5_NEW_POSITIVE_MM;
            s_phaseStartMs = Timebase_GetMillis();
            s_state = ROUTE5_NEW_RAMP_DOWN;
        } else {
            float ratio = (float)elapsedMs / (float)ROUTE5_NEW_RAMP_UP_MS;
            s_targetMm = ROUTE5_NEW_ZERO_MM +
                (ROUTE5_NEW_POSITIVE_MM - ROUTE5_NEW_ZERO_MM) * ratio;
        }
        ZeroControl_SetTargetMm(s_targetMm);
        break;

    case ROUTE5_NEW_RAMP_DOWN:
        /* 线性: +50mm → -50mm over RAMP_DOWN_MS */
        if (elapsedMs >= ROUTE5_NEW_RAMP_DOWN_MS) {
            s_targetMm = ROUTE5_NEW_NEGATIVE_MM;
            s_state = ROUTE5_NEW_HOLD;
            s_stableFrames = 0U;
        } else {
            float ratio = (float)elapsedMs / (float)ROUTE5_NEW_RAMP_DOWN_MS;
            s_targetMm = ROUTE5_NEW_POSITIVE_MM +
                (ROUTE5_NEW_NEGATIVE_MM - ROUTE5_NEW_POSITIVE_MM) * ratio;
        }
        ZeroControl_SetTargetMm(s_targetMm);
        break;

    case ROUTE5_NEW_HOLD: {
        /* 球在 -50mm 附近稳定后判定完成 */
        float posMm = ZeroControl_GetPositionMm();
        float velMmS = ZeroControl_GetVelocityMmS();
        bool stable =
            (absoluteFloat(posMm - ROUTE5_NEW_NEGATIVE_MM) <=
                ROUTE5_NEW_FINAL_TOL_MM) &&
            (absoluteFloat(velMmS) <= ROUTE5_NEW_FINAL_SPEED_MMPS);

        if (stable && (++s_stableFrames >= ROUTE5_NEW_FINAL_FRAMES)) {
            s_state = ROUTE5_NEW_DONE;
            s_stableFrames = 0U;
        } else if (!stable) {
            s_stableFrames = 0U;
        }
        break;
    }

    default:
        break;
    }
}

bool Route5New_IsRunning(void)
{
    return (s_state != ROUTE5_NEW_IDLE) && (s_state != ROUTE5_NEW_DONE);
}

Route5New_State Route5New_GetState(void)
{
    return s_state;
}

float Route5New_GetTargetMm(void)
{
    return s_targetMm;
}

float Route5New_GetPositionMm(void)
{
    return ZeroControl_GetPositionMm();
}
