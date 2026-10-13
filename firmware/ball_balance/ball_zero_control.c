#include "ball_balance/ball_zero_control.h"

#include <stdint.h>

#include "ball_balance/ball_balance.h"
#include "drivers/stepper_emm_v5.h"
#include "drivers/timebase.h"

/* Hardware retained from the verified Emm42_V5 installation. */
#define ZERO_MOTOR_ADDR                    (2U)
#define ZERO_POSITIVE_DIRECTION            (1U)
#define ZERO_NEGATIVE_DIRECTION            (0U)
#define ZERO_MOTOR_RPM                     (300U)
#define ZERO_MOTOR_ACCEL                   (255U)
#define ZERO_PULSES_PER_REV                (12800.0f)

/* 60 FPS vision: filter only when a new camera frame arrives. */
#define ZERO_CONTROL_PERIOD_MS             (5U)
#define ZERO_FRAME_DT_DEFAULT_S            (0.0167f)
#define ZERO_FRAME_DT_MIN_S                (0.008f)
#define ZERO_FRAME_DT_MAX_S                (0.100f)
#define ZERO_POSITION_ALPHA                (0.50f)
#define ZERO_VELOCITY_ALPHA                (0.35f)
#define ZERO_VELOCITY_LIMIT_MM_S           (500.0f)

/* Position PID -> target ball velocity. Integral starts disabled. */
#define ZERO_POS_KP_DEFAULT                (5.00f)
#define ZERO_POS_KI_DEFAULT                (0.00f)
#define ZERO_POS_KD_DEFAULT                (0.10f)
#define ZERO_TARGET_SPEED_LIMIT_MM_S       (60.0f)
#define ZERO_POSITION_INTEGRAL_LIMIT       (100.0f)

/* Velocity PID -> platform angle. Integral starts disabled. */
#define ZERO_VEL_KP_DEFAULT                (0.100f)
#define ZERO_VEL_KI_DEFAULT                (0.000f)
#define ZERO_VEL_KD_DEFAULT                (0.0005f)
#define ZERO_SPEED_INTEGRAL_LIMIT          (200.0f)

#define ZERO_POSITION_DEADBAND_MM          (1.0f)
#define ZERO_ANGLE_LIMIT_DEG               (13.0f)
#define ZERO_ANGLE_SLEW_DEG_PER_TICK       (0.20f)

static bool s_enabled;
static bool s_filterReady;
static bool s_lastCommandValid;
static float s_positionMm;
static float s_targetPositionMm;
static float s_previousPositionMm;
static float s_velocityMmS;
static float s_targetVelocityMmS;
static float s_angleDeg;
static float s_feedforwardAngleDeg;
static float s_angleSlewDegPerTick;
static float s_positionIntegral;
static float s_speedIntegral;
static float s_previousSpeedError;
static float s_posKp;
static float s_posKi;
static float s_posKd;
static float s_velKp;
static float s_velKi;
static float s_velKd;
static uint32_t s_lastControlMs;
static uint32_t s_lastFrameSequence;
static uint32_t s_lastFrameMs;
static uint32_t s_lastCommandPulses;
static uint8_t s_lastCommandDirection;

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

static void resetPidState(void)
{
    s_filterReady = false;
    s_positionMm = 0.0f;
    s_previousPositionMm = 0.0f;
    s_velocityMmS = 0.0f;
    s_targetVelocityMmS = 0.0f;
    s_positionIntegral = 0.0f;
    s_speedIntegral = 0.0f;
    s_previousSpeedError = 0.0f;
    s_lastFrameSequence = BallBalance_GetVisionFrameSequence();
    s_lastFrameMs = 0U;
}

static void commandAngle(float targetAngleDeg)
{
    uint8_t direction;
    uint32_t pulses;

    targetAngleDeg = clampFloat(
        targetAngleDeg, -ZERO_ANGLE_LIMIT_DEG, ZERO_ANGLE_LIMIT_DEG);
    s_angleDeg = moveToward(
        s_angleDeg, targetAngleDeg, s_angleSlewDegPerTick);

    direction = (s_angleDeg >= 0.0f) ?
        ZERO_POSITIVE_DIRECTION : ZERO_NEGATIVE_DIRECTION;
    pulses = (uint32_t) (
        absoluteFloat(s_angleDeg) * ZERO_PULSES_PER_REV / 360.0f + 0.5f);

    if (s_lastCommandValid &&
        (pulses == s_lastCommandPulses) &&
        (direction == s_lastCommandDirection)) {
        return;
    }

    Emm_V5_Pos_Control(
        ZERO_MOTOR_ADDR, direction, ZERO_MOTOR_RPM,
        ZERO_MOTOR_ACCEL, pulses, 1U, false);
    s_lastCommandPulses = pulses;
    s_lastCommandDirection = direction;
    s_lastCommandValid = true;
}

static bool updateVisionSample(float *frameDtOut)
{
    uint32_t sequence = BallBalance_GetVisionFrameSequence();
    uint32_t frameMs;
    float rawPositionMm;
    float rawVelocityMmS;
    float frameDt;

    if (sequence == s_lastFrameSequence) return false;
    s_lastFrameSequence = sequence;
    frameMs = BallBalance_GetVisionFrameMs();
    rawPositionMm = BallBalance_GetVisionPositionMm();

    if (!s_filterReady) {
        s_positionMm = rawPositionMm;
        s_previousPositionMm = rawPositionMm;
        s_velocityMmS = 0.0f;
        frameDt = ZERO_FRAME_DT_DEFAULT_S;
        s_filterReady = true;
    } else {
        frameDt = (float) (frameMs - s_lastFrameMs) * 0.001f;
        if ((frameDt < ZERO_FRAME_DT_MIN_S) ||
            (frameDt > ZERO_FRAME_DT_MAX_S)) {
            frameDt = ZERO_FRAME_DT_DEFAULT_S;
        }
        s_positionMm += ZERO_POSITION_ALPHA *
            (rawPositionMm - s_positionMm);
        rawVelocityMmS =
            (s_positionMm - s_previousPositionMm) / frameDt;
        rawVelocityMmS = clampFloat(
            rawVelocityMmS,
            -ZERO_VELOCITY_LIMIT_MM_S,
            ZERO_VELOCITY_LIMIT_MM_S);
        s_velocityMmS += ZERO_VELOCITY_ALPHA *
            (rawVelocityMmS - s_velocityMmS);
        s_previousPositionMm = s_positionMm;
    }
    s_lastFrameMs = frameMs;
    *frameDtOut = frameDt;
    return true;
}

void ZeroControl_Init(void)
{
    s_enabled = false;
    s_angleDeg = 0.0f;
    s_feedforwardAngleDeg = 0.0f;
    s_angleSlewDegPerTick = ZERO_ANGLE_SLEW_DEG_PER_TICK;
    s_targetPositionMm = 0.0f;
    s_lastCommandValid = false;
    s_posKp = ZERO_POS_KP_DEFAULT;
    s_posKi = ZERO_POS_KI_DEFAULT;
    s_posKd = ZERO_POS_KD_DEFAULT;
    s_velKp = ZERO_VEL_KP_DEFAULT;
    s_velKi = ZERO_VEL_KI_DEFAULT;
    s_velKd = ZERO_VEL_KD_DEFAULT;
    s_lastControlMs = Timebase_GetMillis();
    resetPidState();
}

void ZeroControl_SetEnabled(bool enabled)
{
    s_enabled = enabled;
    resetPidState();
    s_lastControlMs = Timebase_GetMillis();
    s_lastCommandValid = false;

    if (enabled) {
        Emm_V5_En_Control(ZERO_MOTOR_ADDR, true, false);
    } else {
        s_targetVelocityMmS = 0.0f;
        /* Stop means an immediate absolute level target, not one slew step. */
        s_angleDeg = 0.0f;
        commandAngle(0.0f);
    }
}

void ZeroControl_Toggle(void)
{
    ZeroControl_SetEnabled(!s_enabled);
}

void ZeroControl_Update(void)
{
    uint32_t now = Timebase_GetMillis();
    uint32_t elapsedMs = now - s_lastControlMs;
    float frameDt = ZERO_FRAME_DT_DEFAULT_S;
    float controlDt;
    float positionError;
    float speedError;
    float speedErrorDerivative;
    float targetAngleDeg;
    bool newFrame;

    if (elapsedMs < ZERO_CONTROL_PERIOD_MS) return;
    s_lastControlMs = now;
    controlDt = (float) elapsedMs * 0.001f;

    if (!s_enabled) return;
    if (!BallBalance_IsBallVisible()) {
        resetPidState();
        s_targetVelocityMmS = 0.0f;
        commandAngle(s_feedforwardAngleDeg);
        return;
    }

    newFrame = updateVisionSample(&frameDt);
    if (!s_filterReady) {
        commandAngle(0.0f);
        return;
    }

    positionError = s_targetPositionMm - s_positionMm;
    if (absoluteFloat(positionError) <= ZERO_POSITION_DEADBAND_MM) {
        positionError = 0.0f;
    }

    if (newFrame) {
        s_positionIntegral += positionError * frameDt;
        s_positionIntegral = clampFloat(
            s_positionIntegral,
            -ZERO_POSITION_INTEGRAL_LIMIT,
            ZERO_POSITION_INTEGRAL_LIMIT);
    }
    s_targetVelocityMmS =
        s_posKp * positionError +
        s_posKi * s_positionIntegral -
        s_posKd * s_velocityMmS;
    s_targetVelocityMmS = clampFloat(
        s_targetVelocityMmS,
        -ZERO_TARGET_SPEED_LIMIT_MM_S,
        ZERO_TARGET_SPEED_LIMIT_MM_S);

    /* Positive platform angle makes the ball move toward negative position. */
    speedError = s_velocityMmS - s_targetVelocityMmS;
    s_speedIntegral += speedError * controlDt;
    s_speedIntegral = clampFloat(
        s_speedIntegral,
        -ZERO_SPEED_INTEGRAL_LIMIT,
        ZERO_SPEED_INTEGRAL_LIMIT);
    speedErrorDerivative = newFrame ?
        (speedError - s_previousSpeedError) / frameDt : 0.0f;
    if (newFrame) s_previousSpeedError = speedError;

    targetAngleDeg =
        s_velKp * speedError +
        s_velKi * s_speedIntegral +
        s_velKd * speedErrorDerivative +
        s_feedforwardAngleDeg;
    commandAngle(targetAngleDeg);
}

void ZeroControl_SetTargetMm(float targetMm)
{
    s_targetPositionMm = clampFloat(targetMm, -125.0f, 125.0f);
    s_positionIntegral = 0.0f;
    s_speedIntegral = 0.0f;
    s_previousSpeedError = 0.0f;
}

void ZeroControl_SetFeedforwardAngleDeg(float angleDeg)
{
    s_feedforwardAngleDeg = clampFloat(angleDeg, -3.0f, 3.0f);
}

void ZeroControl_SetAngleSlewDegPerTick(float stepDeg)
{
    s_angleSlewDegPerTick = clampFloat(stepDeg, 0.05f, 1.50f);
}

void ZeroControl_AdjustPositionKp(float delta)
{
    if (s_enabled) return;
    s_posKp = clampFloat(s_posKp + delta, 0.0f, 3.0f);
    s_positionIntegral = 0.0f;
}

bool ZeroControl_IsEnabled(void) { return s_enabled; }
float ZeroControl_GetPositionMm(void) { return s_positionMm; }
float ZeroControl_GetTargetMm(void) { return s_targetPositionMm; }
float ZeroControl_GetFeedforwardAngleDeg(void)
{
    return s_feedforwardAngleDeg;
}
float ZeroControl_GetVelocityMmS(void) { return s_velocityMmS; }
float ZeroControl_GetTargetVelocityMmS(void) { return s_targetVelocityMmS; }
float ZeroControl_GetAngleDeg(void) { return s_angleDeg; }
float ZeroControl_GetPositionKp(void) { return s_posKp; }
float ZeroControl_GetPositionKd(void) { return s_posKd; }
float ZeroControl_GetVelocityKp(void) { return s_velKp; }
float ZeroControl_GetVelocityKd(void) { return s_velKd; }
