#include "app/mode_ab20.h"

#include "drivers/encoder_odometry.h"
#include "app/sys_mode.h"
#include "ball_balance/ball_hf_zero.h"
#include "drivers/encoder_odometry.h"
#include "drivers/line_tracker.h"
#include "drivers/motor_tb6612.h"
#include "drivers/timebase.h"

#define MODEAB_TARGET_DISTANCE_MM          (1500.0f)
#define MODEAB_MAX_DUTY_PERCENT            (40)
#define MODEAB_MIN_DRIVE_PERCENT           (12)
#define MODEAB_ACCEL_TIME_MS               (1500U)
#define MODEAB_PRETILT_HOLD_MS              (250U)
#define MODEAB_FINAL_SETTLE_MS             (1200U)
#define MODEAB_START_FEEDFORWARD_DEG        (2.2f)
#define MODEAB_BRAKE_FEEDFORWARD_DEG       (-1.2f)
#define MODEAB_BRAKE_HOLD_MS                (300U)
#define MODEAB_BRAKE_RELEASE_MS             (400U)
#define MODEAB_DECEL_DISTANCE_MM           (700.0f)
#define MODEAB_FINAL_CREEP_DISTANCE_MM     (150.0f)

#define MODEAB_LINE_KP                     (0.55f)
#define MODEAB_LINE_CORRECTION_LIMIT       (12.0f)
#define MODEAB_LOST_LINE_CORRECTION        (12.0f)

static ModeAB_State s_state;
static uint32_t s_phaseStartMs;
static int8_t s_drivePercent;
static int8_t s_lastLineError;
static float s_targetDistanceMm = MODEAB_TARGET_DISTANCE_MM;

static int8_t clampWheelPercent(int16_t value)
{
    if (value > MODEAB_MAX_DUTY_PERCENT) return MODEAB_MAX_DUTY_PERCENT;
    if (value < 0) return 0;
    return (int8_t)value;
}

/* Preserve the requested left/right difference while enforcing the 30%
 * ceiling.  Shift both wheel commands together instead of independently
 * clipping one wheel, so high-speed steering authority is retained. */
static void makeDifferentialCommand(int16_t basePercent, int16_t correction,
                                    int8_t *left, int8_t *right)
{
    int16_t leftCommand = basePercent + correction;
    int16_t rightCommand = basePercent - correction;
    int16_t highest = (leftCommand > rightCommand) ?
        leftCommand : rightCommand;

    if (highest > MODEAB_MAX_DUTY_PERCENT) {
        int16_t excess = highest - MODEAB_MAX_DUTY_PERCENT;
        leftCommand -= excess;
        rightCommand -= excess;
    }
    *left = clampWheelPercent(leftCommand);
    *right = clampWheelPercent(rightCommand);
}

void ModeAB_Init(void)
{
    s_state = MODEAB_IDLE;
    s_phaseStartMs = 0U;
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_targetDistanceMm = MODEAB_TARGET_DISTANCE_MM;
}

void ModeAB_Stop(void)
{
    TB6612_Stop();
    HFZero_SetFeedforwardAngleDeg(0.0f);
    HFZero_SetEnabled(false);
    s_state = MODEAB_IDLE;
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_targetDistanceMm = MODEAB_TARGET_DISTANCE_MM;
}

void ModeAB_OnStartStop(void)
{
    if (AB_GetMode() != MODE_BALL_AB20) return;

    if ((s_state != MODEAB_IDLE) && (s_state != MODEAB_DONE)) {
        ModeAB_Stop();
        return;
    }

    TB6612_Stop();
    EncoderDistance_Reset();
    s_drivePercent = 0;
    s_lastLineError = 0;
    s_targetDistanceMm = MODEAB_TARGET_DISTANCE_MM + EncoderDistance_GetAdjustMm();
    /* Pre-lower the platform before chassis acceleration. Positive platform
     * angle drives the ball in the negative direction in the current setup. */
    HFZero_SetFeedforwardAngleDeg(MODEAB_START_FEEDFORWARD_DEG);
    HFZero_SetEnabled(true);
    s_phaseStartMs = Timebase_GetMillis();
    s_state = MODEAB_WAIT_BALL;
}

void ModeAB_Update(const LineTracker_State *line,
                   int8_t *left, int8_t *right)
{
    float distanceMm;
    float remainingMm;
    float correction;
    uint32_t elapsedMs;
    int16_t basePercent;

    *left = 0;
    *right = 0;

    if ((s_state == MODEAB_IDLE) || (s_state == MODEAB_DONE)) return;

    if (s_state == MODEAB_WAIT_BALL) {
        TB6612_Stop();
        if ((Timebase_GetMillis() - s_phaseStartMs) >=
            MODEAB_PRETILT_HOLD_MS) {
            s_phaseStartMs = Timebase_GetMillis();
            s_state = MODEAB_ACCEL;
            /* Keep acceleration feedforward during ramp to counteract
             * inertial drift of the ball.  Cleared when entering CRUISE. */
            HFZero_SetFeedforwardAngleDeg(MODEAB_START_FEEDFORWARD_DEG);
        }
        return;
    }

    if (s_state == MODEAB_FINAL_BALANCE) {
        uint32_t finalElapsedMs = Timebase_GetMillis() - s_phaseStartMs;
        TB6612_Stop();
        if (finalElapsedMs < MODEAB_BRAKE_HOLD_MS) {
            HFZero_SetFeedforwardAngleDeg(
                MODEAB_BRAKE_FEEDFORWARD_DEG);
        } else if (finalElapsedMs <
                   (MODEAB_BRAKE_HOLD_MS + MODEAB_BRAKE_RELEASE_MS)) {
            float releaseRatio = (float)(finalElapsedMs -
                MODEAB_BRAKE_HOLD_MS) / (float)MODEAB_BRAKE_RELEASE_MS;
            HFZero_SetFeedforwardAngleDeg(
                MODEAB_BRAKE_FEEDFORWARD_DEG * (1.0f - releaseRatio));
        } else {
            HFZero_SetFeedforwardAngleDeg(0.0f);
        }
        if (finalElapsedMs >= MODEAB_FINAL_SETTLE_MS) {
            s_state = MODEAB_DONE;
        }
        return;
    }

    distanceMm = EncoderDistance_GetAverageMm();
    if (distanceMm >= s_targetDistanceMm) {
        TB6612_Stop();
        HFZero_SetFeedforwardAngleDeg(MODEAB_BRAKE_FEEDFORWARD_DEG);
        s_drivePercent = 0;
        s_state = MODEAB_FINAL_BALANCE;
        s_phaseStartMs = Timebase_GetMillis();
        return;
    }

    remainingMm = s_targetDistanceMm - distanceMm;
    elapsedMs = Timebase_GetMillis() - s_phaseStartMs;
    if ((s_state == MODEAB_ACCEL) &&
        (elapsedMs < MODEAB_ACCEL_TIME_MS)) {
        basePercent = MODEAB_MIN_DRIVE_PERCENT +
            (int16_t)(((uint32_t)(MODEAB_MAX_DUTY_PERCENT -
                MODEAB_MIN_DRIVE_PERCENT) * elapsedMs) /
                MODEAB_ACCEL_TIME_MS);
    } else {
        if (s_state == MODEAB_ACCEL) {
            s_state = MODEAB_CRUISE;
            HFZero_SetFeedforwardAngleDeg(0.0f);
        }
        basePercent = MODEAB_MAX_DUTY_PERCENT;
    }

    if (remainingMm <= MODEAB_DECEL_DISTANCE_MM) {
        float spanMm = MODEAB_DECEL_DISTANCE_MM -
            MODEAB_FINAL_CREEP_DISTANCE_MM;
        float brakeRatio =
            1.0f - remainingMm / MODEAB_DECEL_DISTANCE_MM;
        s_state = MODEAB_DECEL;
        if (brakeRatio < 0.0f) brakeRatio = 0.0f;
        if (brakeRatio > 1.0f) brakeRatio = 1.0f;
        HFZero_SetFeedforwardAngleDeg(
            MODEAB_BRAKE_FEEDFORWARD_DEG * brakeRatio);
        if (remainingMm <= MODEAB_FINAL_CREEP_DISTANCE_MM) {
            basePercent = MODEAB_MIN_DRIVE_PERCENT;
        } else {
            basePercent = MODEAB_MIN_DRIVE_PERCENT + (int16_t)(
                (float)(MODEAB_MAX_DUTY_PERCENT -
                    MODEAB_MIN_DRIVE_PERCENT) *
                (remainingMm - MODEAB_FINAL_CREEP_DISTANCE_MM) / spanMm);
        }
    }

    if (line->lineFound != 0U) {
        s_lastLineError = line->error;
        correction = MODEAB_LINE_KP * (float)line->error;
    } else if (s_lastLineError != 0) {
        correction = (s_lastLineError > 0) ?
            MODEAB_LOST_LINE_CORRECTION :
           -MODEAB_LOST_LINE_CORRECTION;
    } else {
        /* Never drive blind when no grey-line direction is known. */
        s_drivePercent = 0;
        TB6612_Stop();
        return;
    }
    if (correction > MODEAB_LINE_CORRECTION_LIMIT)
        correction = MODEAB_LINE_CORRECTION_LIMIT;
    if (correction < -MODEAB_LINE_CORRECTION_LIMIT)
        correction = -MODEAB_LINE_CORRECTION_LIMIT;

    makeDifferentialCommand(basePercent, (int16_t)correction, left, right);
    s_drivePercent = (int8_t)basePercent;
    TB6612_SetSpeed(*left, *right);
}

uint8_t ModeAB_IsRunning(void)
{
    return (uint8_t)((s_state != MODEAB_IDLE) &&
                     (s_state != MODEAB_DONE));
}

ModeAB_State ModeAB_GetState(void)
{
    return s_state;
}

float ModeAB_GetDistanceMm(void)
{
    return EncoderDistance_GetAverageMm();
}

int8_t ModeAB_GetDrivePercent(void)
{
    return s_drivePercent;
}
