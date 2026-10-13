#include "app/mode_loop_arc_stop.h"

#include "app/sys_mode.h"
#include "ball_balance/ball_hf_zero.h"
#include "ball_balance/ball_balance.h"
#include "drivers/encoder_odometry.h"
#include "drivers/line_tracker.h"
#include "drivers/motor_tb6612.h"
#include "drivers/timebase.h"

/* ======================== Distance parameters ======================== */
#define MODE007_DISTANCE_SCALE      (5600.0f / 6140.0f)
#define MODE007_S1_END_MM          (1500.0f * MODE007_DISTANCE_SCALE)
#define MODE007_A1_END_MM          (3070.0f * MODE007_DISTANCE_SCALE)
#define MODE007_S2_END_MM          (4570.0f * MODE007_DISTANCE_SCALE)
#define MODE007_FINISH_MM          (5600.0f)

/* ======================== Speed parameters ======================== */
#define MODE007_MAX_DUTY           (34)
#define MODE007_START_DUTY         (12)
#define MODE007_CURVE_DUTY         (27)
#define MODE007_FINAL_DUTY         (16)
#define MODE007_ACCEL_MS           (3000U)
#define MODE007_DECEL_MM           (520.0f * MODE007_DISTANCE_SCALE)

/* ======================== Line tracking gains ======================== */
#define MODE007_S_KP               (0.45f)
#define MODE007_S_LIM              (10.0f)
#define MODE007_C_KP               (0.55f)
#define MODE007_C_LIM              (13.0f)
#define MODE007_C_BIAS             (2.6f)
#define MODE007_LOST               (13.0f)
#define MODE007_SIGN_TH            (10)
#define MODE007_SIGN_CNT           (20U)

/* ======================== Ball control ======================== */
#define MODE007_PRETILT_MS         (150U)
#define MODE007_FF_START           (1.5f)
#define MODE007_FF_BRAKE           (-1.3f)
#define MODE007_BRAKE_HOLD_MS      (250U)
#define MODE007_BRAKE_RELEASE_MS   (300U)
#define MODE007_FINAL_SETTLE_MS    (600U)
#define MODE007_FAST_ENTER         (3.0f)
#define MODE007_FAST_EXIT          (2.0f)
#define MODE007_SPD_ENTER          (40.0f)
#define MODE007_SPD_EXIT           (20.0f)

/* ======================== Module state ======================== */
static Mode007_State s_state;
static uint32_t s_phaseStartMs;
static int8_t s_drivePercent;
static int8_t s_lastLineError;
static int8_t s_curveDirection;
static int8_t s_curveCandidate;
static uint8_t s_curveConfirmCount;
static uint8_t s_fastBallResponse;
static float s_targetMm;
static float s_finishMm = MODE007_FINISH_MM;

/* ======================== Helpers ======================== */
static float absoluteFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static int8_t clampWheelPercent(int16_t value)
{
    if (value > MODE007_MAX_DUTY) return MODE007_MAX_DUTY;
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

    if (highest > MODE007_MAX_DUTY) {
        int16_t excess = highest - MODE007_MAX_DUTY;
        leftCommand -= excess;
        rightCommand -= excess;
    }
    *left = clampWheelPercent(leftCommand);
    *right = clampWheelPercent(rightCommand);
}

static void updateBallResponseSpeed(void)
{
    float positionAbs = absoluteFloat(HFZero_GetPositionMm() - s_targetMm);
    float velocityAbs = absoluteFloat(HFZero_GetVelocityMmS());

    if (s_fastBallResponse != 0U) {
        if ((positionAbs <= MODE007_FAST_EXIT) &&
            (velocityAbs <= MODE007_SPD_EXIT)) {
            s_fastBallResponse = 0U;
        }
    } else if ((positionAbs >= MODE007_FAST_ENTER) ||
               (velocityAbs >= MODE007_SPD_ENTER)) {
        s_fastBallResponse = 1U;
    }
}

static void learnCurveDirection(int8_t error)
{
    int8_t candidate;

    if (s_curveDirection != 0) return;
    if (error >= MODE007_SIGN_TH) candidate = 1;
    else if (error <= -MODE007_SIGN_TH) candidate = -1;
    else {
        s_curveCandidate = 0;
        s_curveConfirmCount = 0U;
        return;
    }

    if (candidate == s_curveCandidate) {
        if (s_curveConfirmCount < MODE007_SIGN_CNT)
            ++s_curveConfirmCount;
    } else {
        s_curveCandidate = candidate;
        s_curveConfirmCount = 1U;
    }
    if (s_curveConfirmCount >= MODE007_SIGN_CNT)
        s_curveDirection = candidate;
}

/* ======================== API ======================== */

void Mode007_Init(void)
{
    s_state = MODE007_IDLE;
    s_phaseStartMs = 0U;
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_curveDirection = 0;
    s_curveCandidate = 0;
    s_curveConfirmCount = 0U;
    s_fastBallResponse = 0U;
    s_targetMm = 0.0f;
    s_finishMm = MODE007_FINISH_MM;
}

void Mode007_Stop(void)
{
    TB6612_Stop();
    HFZero_SetFeedforwardAngleDeg(0.0f);
    HFZero_SetFeedforwardClampDeg(3.0f);
    HFZero_SetAngleLimitDeg(8.0f);
    HFZero_SetManualSlewDeg(0.0f);
    HFZero_SetStandaloneOscillationMode(false);
    HFZero_SetEnabled(false);
    HFZero_SetTargetMm(0.0f);
    s_state = MODE007_IDLE;
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_curveDirection = 0;
    s_curveCandidate = 0;
    s_curveConfirmCount = 0U;
    s_fastBallResponse = 0U;
}

void Mode007_OnStartStop(void)
{
    if (AB_GetMode() != MODE_007) return;

    if ((s_state != MODE007_IDLE) && (s_state != MODE007_DONE)) {
        Mode007_Stop();
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
    s_finishMm = MODE007_FINISH_MM + EncoderDistance_GetAdjustMm();

    if (BallBalance_IsBallVisible()) s_targetMm = BallBalance_GetVisionPositionMm();
    else s_targetMm = 0.0f;

    HFZero_SetTargetMm(s_targetMm);
    HFZero_SetFeedforwardClampDeg(5.0f);
    HFZero_SetAngleLimitDeg(8.0f);
    HFZero_SetManualSlewDeg(0.60f);
    HFZero_SetFeedforwardAngleDeg(MODE007_FF_START);
    HFZero_SetEnabled(true);
    s_phaseStartMs = Timebase_GetMillis();
    s_state = MODE007_HOLD;
}

void Mode007_Update(const LineTracker_State *line,
                    int8_t *left, int8_t *right)
{
    float distanceMm;
    float remainingMm;
    float correction;
    float kp;
    float correctionLimit;
    uint32_t elapsedMs;
    int16_t basePercent;
    uint8_t onCurve;

    *left = 0;
    *right = 0;

    if ((s_state == MODE007_IDLE) || (s_state == MODE007_DONE)) return;

    /* ---- HOLD: pre-tilt wait ---- */
    if (s_state == MODE007_HOLD) {
        TB6612_Stop();
        if ((Timebase_GetMillis() - s_phaseStartMs) >=
            MODE007_PRETILT_MS) {
            s_phaseStartMs = Timebase_GetMillis();
            s_state = MODE007_DRIVE;
            HFZero_SetFeedforwardAngleDeg(MODE007_FF_START);
        }
        return;
    }

    /* ---- FINAL: ball settle ---- */
    if (s_state == MODE007_FINAL) {
        uint32_t finalMs = Timebase_GetMillis() - s_phaseStartMs;
        TB6612_Stop();
        if (finalMs < MODE007_BRAKE_HOLD_MS) {
            HFZero_SetFeedforwardAngleDeg(MODE007_FF_BRAKE);
        } else if (finalMs <
                   (MODE007_BRAKE_HOLD_MS + MODE007_BRAKE_RELEASE_MS)) {
            float releaseRatio = (float)(finalMs - MODE007_BRAKE_HOLD_MS) /
                (float)MODE007_BRAKE_RELEASE_MS;
            HFZero_SetFeedforwardAngleDeg(
                MODE007_FF_BRAKE * (1.0f - releaseRatio));
        } else {
            HFZero_SetFeedforwardAngleDeg(0.0f);
            updateBallResponseSpeed();
        }
        if (finalMs >= MODE007_FINAL_SETTLE_MS)
            s_state = MODE007_DONE;
        return;
    }

    /* ---- DRIVE: mode6 arc trajectory (0..5600mm) ---- */
    distanceMm = EncoderDistance_GetAverageMm();

    /* Reached A point -> stop and settle. */
    if (distanceMm >= s_finishMm) {
        TB6612_Stop();
        HFZero_SetFeedforwardAngleDeg(MODE007_FF_BRAKE);
        s_drivePercent = 0;
        s_phaseStartMs = Timebase_GetMillis();
        s_state = MODE007_FINAL;
        return;
    }

    onCurve = (uint8_t)(
        (distanceMm >= MODE007_S1_END_MM && distanceMm < MODE007_A1_END_MM) ||
        (distanceMm >= MODE007_S2_END_MM));

    basePercent = onCurve ? MODE007_CURVE_DUTY : MODE007_MAX_DUTY;

    /* Soft start: linear ramp from START_DUTY to MAX_DUTY. */
    elapsedMs = Timebase_GetMillis() - s_phaseStartMs;
    if ((s_state == MODE007_DRIVE) &&
        (elapsedMs < MODE007_ACCEL_MS)) {
        basePercent = MODE007_START_DUTY + (int16_t)(
            (uint32_t)(MODE007_MAX_DUTY - MODE007_START_DUTY) *
            elapsedMs / MODE007_ACCEL_MS);
    } else {
        HFZero_SetFeedforwardAngleDeg(0.0f);
        HFZero_SetManualSlewDeg(0.0f);
        updateBallResponseSpeed();
    }

    /* Curve-end deceleration. */
    remainingMm = MODE007_FINISH_MM - distanceMm;
    if (onCurve && (remainingMm <= MODE007_DECEL_MM)) {
        float ratio = clampFloat(
            remainingMm / MODE007_DECEL_MM, 0.0f, 1.0f);
        basePercent = MODE007_FINAL_DUTY + (int16_t)(
            (float)(MODE007_CURVE_DUTY - MODE007_FINAL_DUTY) * ratio);
        HFZero_SetFeedforwardAngleDeg(
            MODE007_FF_BRAKE * (1.0f - ratio));
    }

    kp = onCurve ? MODE007_C_KP : MODE007_S_KP;
    correctionLimit = onCurve ? MODE007_C_LIM : MODE007_S_LIM;

    if (line->lineFound != 0U) {
        s_lastLineError = line->error;
        if (onCurve != 0U) learnCurveDirection(line->error);
        correction = kp * (float)line->error;
        if ((onCurve != 0U) && (s_curveDirection != 0))
            correction += (float)s_curveDirection * MODE007_C_BIAS;
    } else {
        int8_t recoveryDirection = (s_curveDirection != 0) ?
            s_curveDirection : ((s_lastLineError > 0) ? 1 :
                ((s_lastLineError < 0) ? -1 : 0));
        if (recoveryDirection == 0) {
            s_drivePercent = 0;
            TB6612_Stop();
            return;
        }
        correction = (float)recoveryDirection * MODE007_LOST;
    }
    correction = clampFloat(correction, -correctionLimit, correctionLimit);

    makeDifferentialCommand(basePercent, (int16_t)correction, left, right);
    s_drivePercent = (int8_t)basePercent;
    TB6612_SetSpeed(*left, *right);
}

uint8_t Mode007_IsRunning(void)
{
    return (uint8_t)((s_state != MODE007_IDLE) &&
                     (s_state != MODE007_DONE));
}

Mode007_State Mode007_GetState(void) { return s_state; }
float Mode007_GetTargetMm(void) { return s_targetMm; }
float Mode007_GetDistanceMm(void) { return EncoderDistance_GetAverageMm(); }
float Mode007_GetFinishMm(void) { return s_finishMm; }
int8_t Mode007_GetDrivePercent(void) { return s_drivePercent; }
