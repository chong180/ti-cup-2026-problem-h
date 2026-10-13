#include "app/mode_full_loop.h"

#include "drivers/encoder_odometry.h"
#include "app/sys_mode.h"
#include "ball_balance/ball_zero_control.h"
#include "drivers/encoder_odometry.h"
#include "drivers/motor_tb6612.h"
#include "drivers/timebase.h"

/* Preserve the original 1500:1570:1500:1570 route proportions while
 * calibrating the encoder's total route distance to 5600 mm. */
#define ABCD_DISTANCE_SCALE                 (5600.0f / 6140.0f)
#define ABCD_STRAIGHT_LENGTH_MM             (1500.0f * ABCD_DISTANCE_SCALE)
#define ABCD_ARC_LENGTH_MM                  (1570.0f * ABCD_DISTANCE_SCALE)
#define ABCD_STRAIGHT_1_END_MM              (1500.0f * ABCD_DISTANCE_SCALE)
#define ABCD_ARC_1_END_MM                   (3070.0f * ABCD_DISTANCE_SCALE)
#define ABCD_STRAIGHT_2_END_MM              (4570.0f * ABCD_DISTANCE_SCALE)
#define ABCD_FINISH_MM                      (5600.0f)

#define ABCD_MAX_DUTY_PERCENT               (26)
#define ABCD_START_DUTY_PERCENT             (16)
#define ABCD_CURVE_DUTY_PERCENT             (21)
#define ABCD_FINAL_DUTY_PERCENT             (16)
#define ABCD_START_ACCEL_MS                 (800U)
#define ABCD_TRANSITION_DISTANCE_MM         (200.0f * ABCD_DISTANCE_SCALE)
#define ABCD_FINAL_DECEL_DISTANCE_MM        (520.0f * ABCD_DISTANCE_SCALE)

#define ABCD_STRAIGHT_KP                    (0.45f)
#define ABCD_STRAIGHT_CORRECTION_LIMIT      (10.0f)
#define ABCD_CURVE_KP                       (0.55f)
#define ABCD_CURVE_CORRECTION_LIMIT         (13.0f)
#define ABCD_CURVE_STEER_BIAS               (2.6f)
#define ABCD_LOST_CORRECTION                (13.0f)
#define ABCD_CURVE_SIGN_ERROR_THRESHOLD     (10)
#define ABCD_CURVE_SIGN_CONFIRM_COUNT       (20U)

#define ABCD_PRETILT_HOLD_MS                (200U)
#define ABCD_START_FEEDFORWARD_DEG          (1.3f)
#define ABCD_BRAKE_FEEDFORWARD_DEG          (-1.0f)
#define ABCD_BRAKE_HOLD_MS                  (250U)
#define ABCD_BRAKE_RELEASE_MS               (300U)
#define ABCD_FINAL_SETTLE_MS                (800U)

#define ABCD_START_ANGLE_SLEW_DEG           (0.60f)
#define ABCD_FAST_ANGLE_SLEW_DEG            (0.80f)
#define ABCD_NORMAL_ANGLE_SLEW_DEG          (0.20f)
#define ABCD_FAST_POSITION_ENTER_MM         (3.0f)
#define ABCD_FAST_POSITION_EXIT_MM          (2.0f)
#define ABCD_FAST_SPEED_ENTER_MM_S          (40.0f)
#define ABCD_FAST_SPEED_EXIT_MM_S           (20.0f)

static ABCD_State s_state;
static uint32_t s_phaseStartMs;
static int8_t s_drivePercent;
static int8_t s_lastLineError;
static int8_t s_curveDirection;
static int8_t s_curveCandidate;
static uint8_t s_curveConfirmCount;
static uint8_t s_fastBallResponse;

static float s_finishMm = ABCD_FINISH_MM;

static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int8_t clampWheelPercent(int16_t value)
{
    if (value > ABCD_MAX_DUTY_PERCENT) return ABCD_MAX_DUTY_PERCENT;
    if (value < 0) return 0;
    return (int8_t)value;
}

static float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

static void makeDifferentialCommand(int16_t basePercent, int16_t correction,
                                    int8_t *left, int8_t *right)
{
    int16_t leftCommand = basePercent + correction;
    int16_t rightCommand = basePercent - correction;
    int16_t highest = (leftCommand > rightCommand) ?
        leftCommand : rightCommand;

    if (highest > ABCD_MAX_DUTY_PERCENT) {
        int16_t excess = highest - ABCD_MAX_DUTY_PERCENT;
        leftCommand -= excess;
        rightCommand -= excess;
    }
    *left = clampWheelPercent(leftCommand);
    *right = clampWheelPercent(rightCommand);
}

static void updateBallResponseSpeed(void)
{
    float positionAbs = absoluteFloat(ZeroControl_GetPositionMm());
    float velocityAbs = absoluteFloat(ZeroControl_GetVelocityMmS());

    if (s_fastBallResponse != 0U) {
        if ((positionAbs <= ABCD_FAST_POSITION_EXIT_MM) &&
            (velocityAbs <= ABCD_FAST_SPEED_EXIT_MM_S)) {
            s_fastBallResponse = 0U;
        }
    } else if ((positionAbs >= ABCD_FAST_POSITION_ENTER_MM) ||
               (velocityAbs >= ABCD_FAST_SPEED_ENTER_MM_S)) {
        s_fastBallResponse = 1U;
    }
    ZeroControl_SetAngleSlewDegPerTick(
        (s_fastBallResponse != 0U) ? ABCD_FAST_ANGLE_SLEW_DEG :
                                    ABCD_NORMAL_ANGLE_SLEW_DEG);
}

static void learnCurveDirection(int8_t error)
{
    int8_t candidate;

    if (s_curveDirection != 0) return;
    if (error >= ABCD_CURVE_SIGN_ERROR_THRESHOLD) candidate = 1;
    else if (error <= -ABCD_CURVE_SIGN_ERROR_THRESHOLD) candidate = -1;
    else {
        s_curveCandidate = 0;
        s_curveConfirmCount = 0U;
        return;
    }

    if (candidate == s_curveCandidate) {
        if (s_curveConfirmCount < ABCD_CURVE_SIGN_CONFIRM_COUNT)
            ++s_curveConfirmCount;
    } else {
        s_curveCandidate = candidate;
        s_curveConfirmCount = 1U;
    }
    if (s_curveConfirmCount >= ABCD_CURVE_SIGN_CONFIRM_COUNT)
        s_curveDirection = candidate;
}

void ABCD_Init(void)
{
    s_state = ABCD_IDLE;
    s_phaseStartMs = 0U;
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_curveDirection = 0;
    s_curveCandidate = 0;
    s_curveConfirmCount = 0U;
    s_fastBallResponse = 0U;
    s_finishMm = ABCD_FINISH_MM;
}

void ABCD_Stop(void)
{
    TB6612_Stop();
    ZeroControl_SetFeedforwardAngleDeg(0.0f);
    ZeroControl_SetAngleSlewDegPerTick(ABCD_NORMAL_ANGLE_SLEW_DEG);
    ZeroControl_SetEnabled(false);
    ZeroControl_SetTargetMm(0.0f);
    ABCD_Init();
}

void ABCD_OnStartStop(void)
{
    if (AB_GetMode() != MODE_ABCD) return;
    if ((s_state != ABCD_IDLE) && (s_state != ABCD_DONE)) {
        ABCD_Stop();
        return;
    }

    TB6612_Stop();
    EncoderDistance_Reset();
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_curveDirection = 0;
    s_curveCandidate = 0;
    s_curveConfirmCount = 0U;
    s_fastBallResponse = 1U;
    s_finishMm = ABCD_FINISH_MM + EncoderDistance_GetAdjustMm();
    ZeroControl_SetTargetMm(0.0f);
    ZeroControl_SetAngleSlewDegPerTick(ABCD_START_ANGLE_SLEW_DEG);
    ZeroControl_SetFeedforwardAngleDeg(ABCD_START_FEEDFORWARD_DEG);
    ZeroControl_SetEnabled(true);
    s_phaseStartMs = Timebase_GetMillis();
    s_state = ABCD_PRETILT;
}

void ABCD_Update(const LineTracker_State *line, int8_t *left, int8_t *right)
{
    float distanceMm;
    float remainingMm;
    float correction;
    float kp;
    float correctionLimit;
    int16_t basePercent;
    uint32_t elapsedMs;
    uint8_t onCurve;
    ABCD_State previousState;

    *left = 0;
    *right = 0;
    if ((s_state == ABCD_IDLE) || (s_state == ABCD_DONE)) return;

    if (s_state == ABCD_PRETILT) {
        TB6612_Stop();
        if ((Timebase_GetMillis() - s_phaseStartMs) >=
            ABCD_PRETILT_HOLD_MS) {
            s_phaseStartMs = Timebase_GetMillis();
            s_state = ABCD_STRAIGHT_1;
        }
        return;
    }

    if (s_state == ABCD_FINAL_BALANCE) {
        uint32_t finalMs = Timebase_GetMillis() - s_phaseStartMs;
        TB6612_Stop();
        if (finalMs < ABCD_BRAKE_HOLD_MS) {
            ZeroControl_SetFeedforwardAngleDeg(ABCD_BRAKE_FEEDFORWARD_DEG);
            ZeroControl_SetAngleSlewDegPerTick(ABCD_FAST_ANGLE_SLEW_DEG);
        } else if (finalMs < (ABCD_BRAKE_HOLD_MS + ABCD_BRAKE_RELEASE_MS)) {
            float release = (float)(finalMs - ABCD_BRAKE_HOLD_MS) /
                            (float)ABCD_BRAKE_RELEASE_MS;
            ZeroControl_SetFeedforwardAngleDeg(
                ABCD_BRAKE_FEEDFORWARD_DEG * (1.0f - release));
        } else {
            ZeroControl_SetFeedforwardAngleDeg(0.0f);
            updateBallResponseSpeed();
        }
        if (finalMs >= ABCD_FINAL_SETTLE_MS) s_state = ABCD_DONE;
        return;
    }

    distanceMm = EncoderDistance_GetAverageMm();
    if (distanceMm >= s_finishMm) {
        TB6612_Stop();
        ZeroControl_SetFeedforwardAngleDeg(ABCD_BRAKE_FEEDFORWARD_DEG);
        ZeroControl_SetAngleSlewDegPerTick(ABCD_FAST_ANGLE_SLEW_DEG);
        s_drivePercent = 0;
        s_phaseStartMs = Timebase_GetMillis();
        s_state = ABCD_FINAL_BALANCE;
        return;
    }

    previousState = s_state;
    if (distanceMm < ABCD_STRAIGHT_1_END_MM) s_state = ABCD_STRAIGHT_1;
    else if (distanceMm < ABCD_ARC_1_END_MM) s_state = ABCD_ARC_1;
    else if (distanceMm < ABCD_STRAIGHT_2_END_MM) s_state = ABCD_STRAIGHT_2;
    else s_state = ABCD_ARC_2;

    if (((s_state == ABCD_ARC_1) || (s_state == ABCD_ARC_2)) &&
        (s_state != previousState)) {
        s_curveDirection = 0;
        s_curveCandidate = 0;
        s_curveConfirmCount = 0U;
    }

    onCurve = (uint8_t)((s_state == ABCD_ARC_1) ||
                        (s_state == ABCD_ARC_2));
    basePercent = onCurve ? ABCD_CURVE_DUTY_PERCENT :
                            ABCD_MAX_DUTY_PERCENT;

    /* Soft start on the first straight. */
    elapsedMs = Timebase_GetMillis() - s_phaseStartMs;
    if ((s_state == ABCD_STRAIGHT_1) &&
        (elapsedMs < ABCD_START_ACCEL_MS)) {
        basePercent = ABCD_START_DUTY_PERCENT + (int16_t)(
            (uint32_t)(ABCD_MAX_DUTY_PERCENT - ABCD_START_DUTY_PERCENT) *
            elapsedMs / ABCD_START_ACCEL_MS);
        ZeroControl_SetAngleSlewDegPerTick(ABCD_START_ANGLE_SLEW_DEG);
    } else {
        ZeroControl_SetFeedforwardAngleDeg(0.0f);
        updateBallResponseSpeed();
    }

    /* Slow smoothly before each arc and accelerate smoothly after arc 1. */
    if ((s_state == ABCD_STRAIGHT_1) &&
        (distanceMm > ABCD_STRAIGHT_1_END_MM - ABCD_TRANSITION_DISTANCE_MM)) {
        float ratio = (ABCD_STRAIGHT_1_END_MM - distanceMm) /
                      ABCD_TRANSITION_DISTANCE_MM;
        basePercent = ABCD_CURVE_DUTY_PERCENT + (int16_t)(
            (float)(ABCD_MAX_DUTY_PERCENT - ABCD_CURVE_DUTY_PERCENT) *
            clampFloat(ratio, 0.0f, 1.0f));
    } else if (s_state == ABCD_STRAIGHT_2) {
        float fromArcMm = distanceMm - ABCD_ARC_1_END_MM;
        float toArcMm = ABCD_STRAIGHT_2_END_MM - distanceMm;
        if (fromArcMm < ABCD_TRANSITION_DISTANCE_MM) {
            basePercent = ABCD_CURVE_DUTY_PERCENT + (int16_t)(
                (float)(ABCD_MAX_DUTY_PERCENT - ABCD_CURVE_DUTY_PERCENT) *
                clampFloat(fromArcMm / ABCD_TRANSITION_DISTANCE_MM,
                           0.0f, 1.0f));
        } else if (toArcMm < ABCD_TRANSITION_DISTANCE_MM) {
            basePercent = ABCD_CURVE_DUTY_PERCENT + (int16_t)(
                (float)(ABCD_MAX_DUTY_PERCENT - ABCD_CURVE_DUTY_PERCENT) *
                clampFloat(toArcMm / ABCD_TRANSITION_DISTANCE_MM,
                           0.0f, 1.0f));
        }
    }

    /* Final arc: proportionally scaled deceleration and brake-FF ramp. */
    remainingMm = s_finishMm - distanceMm;
    if ((s_state == ABCD_ARC_2) &&
        (remainingMm <= ABCD_FINAL_DECEL_DISTANCE_MM)) {
        float ratio = clampFloat(
            remainingMm / ABCD_FINAL_DECEL_DISTANCE_MM, 0.0f, 1.0f);
        basePercent = ABCD_FINAL_DUTY_PERCENT + (int16_t)(
            (float)(ABCD_CURVE_DUTY_PERCENT - ABCD_FINAL_DUTY_PERCENT) *
            ratio);
        ZeroControl_SetFeedforwardAngleDeg(
            ABCD_BRAKE_FEEDFORWARD_DEG * (1.0f - ratio));
    }

    kp = onCurve ? ABCD_CURVE_KP : ABCD_STRAIGHT_KP;
    correctionLimit = onCurve ? ABCD_CURVE_CORRECTION_LIMIT :
                                ABCD_STRAIGHT_CORRECTION_LIMIT;
    if (line->lineFound != 0U) {
        s_lastLineError = line->error;
        if (onCurve != 0U) learnCurveDirection(line->error);
        correction = kp * (float)line->error;
        if ((onCurve != 0U) && (s_curveDirection != 0))
            correction += (float)s_curveDirection * ABCD_CURVE_STEER_BIAS;
    } else {
        int8_t recoveryDirection = (s_curveDirection != 0) ?
            s_curveDirection : ((s_lastLineError > 0) ? 1 :
                                ((s_lastLineError < 0) ? -1 : 0));
        if (recoveryDirection == 0) {
            s_drivePercent = 0;
            TB6612_Stop();
            return;
        }
        correction = (float)recoveryDirection * ABCD_LOST_CORRECTION;
    }
    correction = clampFloat(correction, -correctionLimit, correctionLimit);
    makeDifferentialCommand(basePercent, (int16_t)correction, left, right);
    s_drivePercent = (int8_t)basePercent;
    TB6612_SetSpeed(*left, *right);
}

uint8_t ABCD_IsRunning(void)
{
    return (uint8_t)((s_state != ABCD_IDLE) && (s_state != ABCD_DONE));
}

ABCD_State ABCD_GetState(void) { return s_state; }
float ABCD_GetDistanceMm(void) { return EncoderDistance_GetAverageMm(); }
int8_t ABCD_GetDrivePercent(void) { return s_drivePercent; }
int8_t ABCD_GetCurveDirection(void) { return s_curveDirection; }
