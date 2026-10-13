#include "ball_balance/ball_hf_zero.h"

#include <stdint.h>

#include "ball_balance/ball_balance.h"
#include "drivers/stepper_emm_v5.h"
#include "drivers/timebase.h"

#define HF_MOTOR_ADDR                       (2U)
#define HF_POSITIVE_DIRECTION               (1U)
#define HF_NEGATIVE_DIRECTION               (0U)
#define HF_MOTOR_RPM                        (900U)
#define HF_MOTOR_ACCEL                      (255U)
#define HF_PULSES_PER_REV                   (12800.0f)

#define HF_CONTROL_PERIOD_MS                (5U)
#define HF_FRAME_DT_DEFAULT_S               (0.0167f)
#define HF_FRAME_DT_MIN_S                   (0.008f)
#define HF_FRAME_DT_MAX_S                   (0.100f)
#define HF_ALPHA                            (0.75f)
#define HF_BETA                             (0.12f)
#define HF_VELOCITY_LIMIT_MM_S              (500.0f)

#define HF_POSITION_GAIN_DEFAULT            (0.35f)
#define HF_FAR_VELOCITY_GAIN                (0.025f)
#define HF_NEAR_POSITION_GAIN_SCALE         (0.45f)
#define HF_NEAR_VELOCITY_GAIN               (0.100f)
#define HF_ANGLE_LIMIT_DEG                  (8.0f)
#define HF_STANDALONE_ANGLE_LIMIT_DEG       (15.0f)
#define HF_STANDALONE_FAST_SLEW_DEG         (3.0f)
#define HF_STANDALONE_NEAR_SLEW_DEG         (1.5f)
#define HF_STANDALONE_FAR_VELOCITY_GAIN     (0.060f)
#define HF_STANDALONE_NEAR_VELOCITY_GAIN    (0.180f)
#define HF_FAST_SLEW_DEG_PER_TICK           (1.00f)
#define HF_NEAR_SLEW_DEG_PER_TICK           (0.80f)
#define HF_HOLD_SLEW_DEG_PER_TICK           (0.20f)
#define HF_FAST_POSITION_ENTER_MM           (8.0f)
#define HF_FAST_POSITION_EXIT_MM            (6.0f)
#define HF_CENTER_SPEED_DEADBAND_MM_S        (10.0f)
#define HF_CENTER_DEADBAND_MM               (0.8f)
#define HF_CENTER_LATCH_POSITION_MM          (0.8f)
#define HF_CENTER_RELEASE_POSITION_MM        (2.0f)
#define HF_CENTER_LATCH_SPEED_MM_S          (10.0f)
#define HF_CENTER_LATCH_FRAMES               (4U)

#define HF_PREDICT_AFTER_LOSS_MS            (80U)
#define HF_LEVEL_AFTER_LOSS_MS              (150U)
#define HF_REACQUIRE_FRAMES                 (2U)

static bool s_enabled;
static bool s_filterReady;
static bool s_fastResponse;
static bool s_lastCommandValid;
static HFZero_State s_state;
static float s_positionMm;
static float s_velocityMmS;
static float s_angleDeg;
static float s_positionGain;
static float s_targetMm;
static float s_feedforwardAngleDeg;
static float s_feedforwardClampDeg = 3.0f;
static float s_angleLimitDeg = 8.0f;
static float s_manualSlewDeg = 0.0f;
static bool s_standaloneOscillationMode;
static uint32_t s_lastControlMs;
static uint32_t s_lastFrameMs;
static uint32_t s_lastFrameSequence;
static uint32_t s_lastCommandPulses;
static uint8_t s_lastCommandDirection;
static uint8_t s_reacquireCount;
static uint8_t s_centerStableFrames;
static bool s_centerLatched;

static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static float moveToward(float current, float target, float step)
{
    if (current < target) {
        current += step;
        return (current > target) ? target : current;
    }
    if (current > target) {
        current -= step;
        return (current < target) ? target : current;
    }
    return current;
}

static void commandAngle(float targetAngleDeg, float slewDeg)
{
    uint8_t direction;
    uint32_t pulses;
    float angleLimitDeg = s_standaloneOscillationMode ?
        HF_STANDALONE_ANGLE_LIMIT_DEG : s_angleLimitDeg;

    targetAngleDeg = clampFloat(
        targetAngleDeg, -angleLimitDeg, angleLimitDeg);
    s_angleDeg = moveToward(s_angleDeg, targetAngleDeg, slewDeg);
    direction = (s_angleDeg >= 0.0f) ?
        HF_POSITIVE_DIRECTION : HF_NEGATIVE_DIRECTION;
    pulses = (uint32_t)(absoluteFloat(s_angleDeg) *
        HF_PULSES_PER_REV / 360.0f + 0.5f);

    if (s_lastCommandValid &&
        (pulses == s_lastCommandPulses) &&
        (direction == s_lastCommandDirection)) return;

    Emm_V5_Pos_Control(HF_MOTOR_ADDR, direction, HF_MOTOR_RPM,
        HF_MOTOR_ACCEL, pulses, 1U, false);
    s_lastCommandPulses = pulses;
    s_lastCommandDirection = direction;
    s_lastCommandValid = true;
}

static void resetEstimator(void)
{
    s_filterReady = false;
    s_fastResponse = false;
    s_positionMm = 0.0f;
    s_velocityMmS = 0.0f;
    s_lastFrameMs = 0U;
    s_lastFrameSequence = BallBalance_GetVisionFrameSequence();
    s_reacquireCount = 0U;
    s_centerStableFrames = 0U;
    s_centerLatched = false;
}

static bool incorporateNewFrame(void)
{
    uint32_t sequence = BallBalance_GetVisionFrameSequence();
    uint32_t frameMs;
    float frameDt;
    float residual;

    if (sequence == s_lastFrameSequence) return false;
    s_lastFrameSequence = sequence;
    frameMs = BallBalance_GetVisionFrameMs();

    if (!s_filterReady) {
        s_positionMm = BallBalance_GetVisionPositionMm();
        s_velocityMmS = 0.0f;
        s_filterReady = true;
        s_reacquireCount = 1U;
    } else {
        frameDt = (float)(frameMs - s_lastFrameMs) * 0.001f;
        if ((frameDt < HF_FRAME_DT_MIN_S) ||
            (frameDt > HF_FRAME_DT_MAX_S)) {
            frameDt = HF_FRAME_DT_DEFAULT_S;
        }
        residual = BallBalance_GetVisionPositionMm() - s_positionMm;
        s_positionMm += HF_ALPHA * residual;
        s_velocityMmS += HF_BETA * residual / frameDt;
        s_velocityMmS = clampFloat(s_velocityMmS,
            -HF_VELOCITY_LIMIT_MM_S, HF_VELOCITY_LIMIT_MM_S);
        if (s_reacquireCount < HF_REACQUIRE_FRAMES) ++s_reacquireCount;
    }
    s_lastFrameMs = frameMs;
    return true;
}

void HFZero_Init(void)
{
    s_enabled = false;
    s_state = HF_ZERO_IDLE;
    s_angleDeg = 0.0f;
    s_positionGain = HF_POSITION_GAIN_DEFAULT;
    s_targetMm = 0.0f;
    s_feedforwardAngleDeg = 0.0f;
    s_standaloneOscillationMode = false;
    s_lastCommandValid = false;
    s_lastControlMs = Timebase_GetMillis();
    resetEstimator();
}

void HFZero_SetEnabled(bool enabled)
{
    s_enabled = enabled;
    s_lastControlMs = Timebase_GetMillis();
    s_lastCommandValid = false;
    resetEstimator();

    if (enabled) {
        s_state = HF_ZERO_ACQUIRE;
        Emm_V5_En_Control(HF_MOTOR_ADDR, true, false);
    } else {
        s_state = HF_ZERO_IDLE;
        s_angleDeg = 0.0f;
        commandAngle(0.0f, HF_FAST_SLEW_DEG_PER_TICK);
    }
}

void HFZero_Toggle(void)
{
    HFZero_SetEnabled(!s_enabled);
}

void HFZero_Update(void)
{
    uint32_t now = Timebase_GetMillis();
    uint32_t elapsedMs = now - s_lastControlMs;
    uint32_t frameAgeMs;
    float controlDt;
    float targetAngleDeg;
    float slewDeg;
    bool newFrame;

    if (elapsedMs < HF_CONTROL_PERIOD_MS) return;
    s_lastControlMs = now;
    if (!s_enabled) return;

    controlDt = (float)elapsedMs * 0.001f;
    if (s_filterReady) {
        s_positionMm += s_velocityMmS * controlDt;
        s_positionMm = clampFloat(s_positionMm, -125.0f, 125.0f);
    }
    newFrame = incorporateNewFrame();

    if (!s_filterReady || (s_lastFrameMs == 0U)) {
        s_state = HF_ZERO_ACQUIRE;
        commandAngle(s_feedforwardAngleDeg, HF_HOLD_SLEW_DEG_PER_TICK);
        return;
    }

    frameAgeMs = now - s_lastFrameMs;
    if (frameAgeMs > HF_LEVEL_AFTER_LOSS_MS) {
        s_state = HF_ZERO_LOST;
        resetEstimator();
        commandAngle(s_feedforwardAngleDeg, HF_HOLD_SLEW_DEG_PER_TICK);
        return;
    }
    if (frameAgeMs > HF_PREDICT_AFTER_LOSS_MS) {
        s_state = HF_ZERO_LOST;
        s_velocityMmS = 0.0f;
        commandAngle(s_feedforwardAngleDeg, HF_HOLD_SLEW_DEG_PER_TICK);
        return;
    }
    if (s_reacquireCount < HF_REACQUIRE_FRAMES) {
        s_state = HF_ZERO_ACQUIRE;
        commandAngle(s_feedforwardAngleDeg, HF_HOLD_SLEW_DEG_PER_TICK);
        return;
    }

    if (s_centerLatched) {
        if (absoluteFloat(s_positionMm - s_targetMm) >=
            HF_CENTER_RELEASE_POSITION_MM) {
            s_centerLatched = false;
            s_centerStableFrames = 0U;
        } else {
            s_state = HF_ZERO_HOLD;
            commandAngle(s_feedforwardAngleDeg,
                HF_HOLD_SLEW_DEG_PER_TICK);
            return;
        }
    }
    if (newFrame) {
        if ((absoluteFloat(s_positionMm - s_targetMm) <=
                HF_CENTER_LATCH_POSITION_MM) &&
            (absoluteFloat(s_velocityMmS) <=
                HF_CENTER_LATCH_SPEED_MM_S)) {
            if (s_centerStableFrames < HF_CENTER_LATCH_FRAMES)
                ++s_centerStableFrames;
            if (s_centerStableFrames >= HF_CENTER_LATCH_FRAMES) {
                s_centerLatched = true;
                s_state = HF_ZERO_HOLD;
                commandAngle(s_feedforwardAngleDeg,
                    HF_HOLD_SLEW_DEG_PER_TICK);
                return;
            }
        } else {
            s_centerStableFrames = 0U;
        }
    }

    if (s_fastResponse) {
        if (absoluteFloat(s_positionMm - s_targetMm) <= HF_FAST_POSITION_EXIT_MM) {
            s_fastResponse = false;
        }
    } else if (absoluteFloat(s_positionMm - s_targetMm) >=
               HF_FAST_POSITION_ENTER_MM) {
        s_fastResponse = true;
    }

    if ((absoluteFloat(s_positionMm - s_targetMm) <= HF_CENTER_DEADBAND_MM) &&
        (absoluteFloat(s_velocityMmS) <=
            HF_CENTER_SPEED_DEADBAND_MM_S)) {
        targetAngleDeg = s_feedforwardAngleDeg;
        slewDeg = HF_HOLD_SLEW_DEG_PER_TICK;
    } else {
        /* Positive angle drives the ball toward negative position. */
        if (s_fastResponse) {
            targetAngleDeg = s_positionGain * (s_positionMm - s_targetMm) +
                (s_standaloneOscillationMode ?
                    HF_STANDALONE_FAR_VELOCITY_GAIN :
                    HF_FAR_VELOCITY_GAIN) * s_velocityMmS +
                s_feedforwardAngleDeg;
            slewDeg = s_standaloneOscillationMode ?
                HF_STANDALONE_FAST_SLEW_DEG :
                HF_FAST_SLEW_DEG_PER_TICK;
        } else {
            /* Near zero: weaker restoring force and stronger velocity
             * damping prevent repeated high-speed crossings. */
            targetAngleDeg =
                (s_positionGain * HF_NEAR_POSITION_GAIN_SCALE) *
                    (s_positionMm - s_targetMm) +
                (s_standaloneOscillationMode ?
                    HF_STANDALONE_NEAR_VELOCITY_GAIN :
                    HF_NEAR_VELOCITY_GAIN) * s_velocityMmS +
                s_feedforwardAngleDeg;
            slewDeg = s_standaloneOscillationMode ?
                HF_STANDALONE_NEAR_SLEW_DEG :
                HF_NEAR_SLEW_DEG_PER_TICK;
        }
    }
    s_state = s_fastResponse ? HF_ZERO_FAST : HF_ZERO_HOLD;
    if (s_manualSlewDeg > 0.0f) slewDeg = s_manualSlewDeg;
    commandAngle(targetAngleDeg, slewDeg);
}

void HFZero_AdjustPositionGain(float delta)
{
    if (s_enabled) return;
    s_positionGain = clampFloat(s_positionGain + delta, 0.05f, 1.00f);
}

void HFZero_SetTargetMm(float mm)
{
    s_targetMm = mm;
}

void HFZero_SetPositionGain(float gain)
{
    s_positionGain = clampFloat(gain, 0.05f, 1.00f);
}

void HFZero_SetFeedforwardAngleDeg(float angleDeg)
{
    s_feedforwardAngleDeg = clampFloat(angleDeg,
        -s_feedforwardClampDeg, s_feedforwardClampDeg);
}

void HFZero_SetFeedforwardClampDeg(float clampDeg)
{
    s_feedforwardClampDeg = clampDeg;
}

void HFZero_SetAngleLimitDeg(float limitDeg)
{
    s_angleLimitDeg = limitDeg;
}

void HFZero_SetManualSlewDeg(float slewDeg)
{
    s_manualSlewDeg = slewDeg;
}

void HFZero_SetStandaloneOscillationMode(bool enabled)
{
    s_standaloneOscillationMode = enabled;
}

bool HFZero_IsEnabled(void) { return s_enabled; }
HFZero_State HFZero_GetState(void) { return s_state; }
float HFZero_GetPositionMm(void) { return s_positionMm; }
float HFZero_GetVelocityMmS(void) { return s_velocityMmS; }
float HFZero_GetAngleDeg(void) { return s_angleDeg; }
float HFZero_GetPositionGain(void) { return s_positionGain; }
float HFZero_GetFeedforwardAngleDeg(void)
{
    return s_feedforwardAngleDeg;
}
